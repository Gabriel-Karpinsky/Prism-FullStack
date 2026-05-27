// EdgeDaemon: top-level orchestration of the Pi-native hardware stack.
//
// Responsibilities:
//   * Own the GPIO backend, LIDAR sensor, MotionController and SafetySupervisor.
//   * Maintain a thread-safe Snapshot that the HTTP layer serialises for the Go
//     control-api. Position (yaw/pitch) is resolved lazily from MotionController
//     at snapshot time so it always reflects the latest committed state.
//   * Run the scan state machine on a dedicated worker thread. The worker
//     calls MotionController::MoveTo synchronously (blocks until the pigpio
//     DMA waveform completes or is aborted), pulses the LIDAR trigger and
//     records the cell. Pause/resume are cooperative; stop_scan and estop
//     trigger motion aborts.
//
// Lock discipline: mutex_ guards state_ and scan_state_. Never held across
// motion_->MoveTo or lidar_->ReadDistanceMeters calls — those may block for
// many milliseconds, and HTTP reads of GetSnapshot must remain responsive.

#include "edge_daemon.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace edge {
namespace {

std::string CurrentTimestamp() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const std::time_t t = clock::to_time_t(now);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &t);
#else
  gmtime_r(&t, &utc);
#endif
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
double Round(double v, double scale)         { return std::round(v * scale) / scale; }

}  // namespace

EdgeDaemon::EdgeDaemon(Config config) : config_(std::move(config)) {
  gpio_   = CreateGpioBackend(config_);
  lidar_  = CreateLidarSensor(config_);
  motion_ = std::make_unique<MotionController>(config_, *gpio_);
  safety_ = std::make_unique<SafetySupervisor>(config_, *motion_);

  // The scan-grid soft limits default to the motion envelope. Operators can
  // tighten them at runtime; widening beyond the motion envelope is bounded
  // by MotionController itself when planning moves.
  state_.scan_settings.yaw_min   = config_.motion.yaw.min_deg;
  state_.scan_settings.yaw_max   = config_.motion.yaw.max_deg;
  state_.scan_settings.pitch_min = config_.motion.pitch.min_deg;
  state_.scan_settings.pitch_max = config_.motion.pitch.max_deg;
  state_.scan_settings.sweep_speed_deg_per_sec =
      std::min(config_.motion.yaw.max_speed_deg_s, config_.motion.pitch.max_speed_deg_s);

  ApplyResolutionLocked(state_.scan_settings.resolution);
  ResetScanLocked();

  AddLogLocked("system", "info", "Edge daemon initialised.");
  AddLogLocked("motion", "info", std::string("GPIO backend: ") + gpio_->Name() + ".");
  AddLogLocked("sensor", "info", std::string("Lidar source: ") + lidar_->Name() + ".");
}

EdgeDaemon::~EdgeDaemon() { Stop(); }

bool EdgeDaemon::Start() {
  {
    std::scoped_lock lock(mutex_);
    if (running_) return true;

    std::string err;
    if (!gpio_->Initialize(err)) {
      state_.connected = false;
      state_.faults = {"GPIO backend init failed: " + err};
      AddLogLocked("motion", "error", state_.faults.front());
      return false;
    }
    if (!lidar_->Initialize()) {
      // Lidar failure isn't fatal at boot — the scan worker surfaces it per-cell.
      AddLogLocked("sensor", "warn", "Lidar init warning: " + lidar_->last_error());
    }

    state_.connected = true;
    running_ = true;
    AddLogLocked("system", "info", "Edge daemon running.");
  }
  motion_->SetEnabled(true);
  safety_->Start();
  return true;
}

void EdgeDaemon::Stop() {
  bool was_running = false;
  {
    std::scoped_lock lock(mutex_);
    was_running = running_.exchange(false);
    if (scan_state_ != ScanState::Idle) scan_state_ = ScanState::Stopping;
  }
  scan_cv_.notify_all();
  if (motion_) motion_->AbortMotion();
  if (scan_worker_.joinable()) scan_worker_.join();
  if (move_worker_.joinable()) move_worker_.join();  // B8: reap any async jog/home
  if (safety_) safety_->Stop();
  if (motion_ && was_running) motion_->SetEnabled(false);
  if (gpio_   && was_running) gpio_->Shutdown();
}

bool EdgeDaemon::Healthy() const {
  return running_.load() && safety_ && !safety_->faulted();
}

Snapshot EdgeDaemon::GetSnapshot() const {
  // Any in-flight HTTP request proves the host (control-api) is alive, so
  // state polls keep the safety watchdog fed even when no motion command
  // is being issued. Without this, a freshly booted daemon with no client
  // commanding it (just polling) would trip host_watchdog after 1500 ms
  // and latch a fault before the operator clicks anything.
  if (safety_) safety_->Heartbeat();

  std::scoped_lock lock(mutex_);
  Snapshot s = state_;
  s.yaw   = Round(motion_->yaw_deg(),   100.0);
  s.pitch = Round(motion_->pitch_deg(), 100.0);

  const FaultCode code = safety_->fault_code();
  if (code != FaultCode::None) {
    const std::string tag = std::string(FaultCodeName(code)) + ": " + safety_->fault_reason();
    if (s.faults.empty() || s.faults.front() != tag) {
      s.faults.insert(s.faults.begin(), tag);
    }
    s.mode = "fault";
    s.connected = false;
  }
  return s;
}

Config EdgeDaemon::GetConfig() const {
  std::scoped_lock lock(mutex_);
  Config c = config_;
  c.motion = motion_->motion_config();
  return c;
}

MotionConfig EdgeDaemon::GetMotionConfig() const { return motion_->motion_config(); }

std::string EdgeDaemon::UpdateMotionConfig(const MotionConfig& motion) {
  if (auto err = ValidateMotionConfig(motion); !err.empty()) return err;

  // Hot-apply requires the current physical position to already lie inside the
  // new envelope; otherwise the next move would immediately clamp and lurch.
  const double yaw   = motion_->yaw_deg();
  const double pitch = motion_->pitch_deg();
  if (yaw < motion.yaw.min_deg || yaw > motion.yaw.max_deg) {
    return "current yaw is outside the proposed envelope; home the gantry first";
  }
  if (pitch < motion.pitch.min_deg || pitch > motion.pitch.max_deg) {
    return "current pitch is outside the proposed envelope; home the gantry first";
  }

  motion_->SetMotionConfig(motion);

  std::scoped_lock lock(mutex_);
  config_.motion = motion;
  state_.scan_settings.yaw_min   = motion.yaw.min_deg;
  state_.scan_settings.yaw_max   = motion.yaw.max_deg;
  state_.scan_settings.pitch_min = motion.pitch.min_deg;
  state_.scan_settings.pitch_max = motion.pitch.max_deg;
  state_.scan_settings.sweep_speed_deg_per_sec =
      std::min(motion.yaw.max_speed_deg_s, motion.pitch.max_speed_deg_s);
  AddLogLocked("config", "info", "Motion envelope updated.");

  if (!config_.config_file_path.empty()) {
    try {
      SaveConfigToFile(config_, config_.config_file_path);
    } catch (const std::exception& e) {
      AddLogLocked("config", "warn", std::string("Persist failed: ") + e.what());
      // Applied in-memory; persistence failure is non-fatal.
    }
  }
  return {};
}

Snapshot EdgeDaemon::ExecuteCommand(const CommandRequest& request, std::string& error_message) {
  error_message.clear();
  safety_->Heartbeat();

  const std::string& cmd = request.command;

  // ESTOP and clear_fault are always accepted, even while the system is faulted.
  if (cmd == "estop") {
    safety_->TriggerEStop("operator e-stop");
    std::thread prev_scan;
    std::thread prev_move;
    {
      std::scoped_lock lock(mutex_);
      state_.mode = "fault";
      if (state_.faults.empty()) state_.faults = {"Emergency stop asserted."};
      if (scan_state_ != ScanState::Idle) {
        scan_state_ = ScanState::Stopping;
        prev_scan = std::move(scan_worker_);
      }
      if (manual_move_active_) prev_move = std::move(move_worker_);  // B8: stop an async jog/home too
      AddLogLocked("safety", "error", "Emergency stop triggered.");
    }
    motion_->AbortMotion();  // halt any in-flight motion (scan move or manual jog/home)
    scan_cv_.notify_all();
    if (prev_scan.joinable()) prev_scan.join();
    if (prev_move.joinable()) prev_move.join();
    return GetSnapshot();
  }

  if (cmd == "clear_fault") {
    safety_->ClearFault();
    std::scoped_lock lock(mutex_);
    state_.faults.clear();
    state_.mode = "idle";
    state_.connected = true;
    scan_state_ = ScanState::Idle;
    AddLogLocked("safety", "info", "Fault cleared.");
    motion_->SetEnabled(true);
    return state_;
  }

  if (safety_->faulted()) {
    error_message = "Clear the fault before issuing commands.";
    std::scoped_lock lock(mutex_);
    return state_;
  }

  if (cmd == "connect") {
    std::scoped_lock lock(mutex_);
    state_.connected = true;
    AddLogLocked("motion", "info", "Connect acknowledged.");
    return state_;
  }

  if (cmd == "set_resolution") {
    std::scoped_lock lock(mutex_);
    if (scan_state_ != ScanState::Idle) {
      error_message = "Stop the current scan before changing resolution.";
      return state_;
    }
    ApplyResolutionLocked(request.resolution.empty() ? state_.scan_settings.resolution
                                                     : request.resolution);
    AddLogLocked("scanner", "info", "Scan resolution set to " + state_.scan_settings.resolution + ".");
    return state_;
  }

  if (cmd == "home") {
    std::thread old_move;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before homing.";
        return state_;
      }
      if (manual_move_active_) {
        error_message = "A move is already in progress.";
        return state_;
      }
      old_move = std::move(move_worker_);  // reap the finished previous move (if any)
      manual_move_active_ = true;
      state_.mode = "manual";
      AddLogLocked("motion", "info", "Homing to (0, 0).");
    }
    // B8: run the move off the HTTP thread and return immediately. The UI polls
    // /api/state and sees mode flip "manual" -> "idle" (or "fault") on completion.
    if (old_move.joinable()) old_move.join();
    move_worker_ = std::thread(&EdgeDaemon::ManualMoveWorker, this, 0.0, 0.0, true);
    return GetSnapshot();
  }

  if (cmd == "jog") {
    if (request.axis != "yaw" && request.axis != "pitch") {
      error_message = "Unsupported jog axis.";
      std::scoped_lock lock(mutex_);
      return state_;
    }
    double target_yaw;
    double target_pitch;
    std::thread old_move;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before jogging.";
        return state_;
      }
      if (manual_move_active_) {
        error_message = "A move is already in progress.";
        return state_;
      }
      // B6: refuse to jog from an untrusted position (an aborted move left
      // tracking stale). Friendly message + no fault latch — the operator just
      // needs to re-home to re-establish the datum.
      if (!motion_->yaw_position_known() || !motion_->pitch_position_known()) {
        error_message = "Position unknown after an aborted move. Home the gantry before jogging.";
        return state_;
      }
      target_yaw   = motion_->yaw_deg();
      target_pitch = motion_->pitch_deg();
      if (request.axis == "yaw") target_yaw   += request.delta;
      else                       target_pitch += request.delta;
      old_move = std::move(move_worker_);
      manual_move_active_ = true;
      state_.mode = "manual";
      AddLogLocked("motion", "info", std::string("Jog ") + request.axis + ".");
    }
    // B8: offload to the move worker; return immediately (see "home" above).
    if (old_move.joinable()) old_move.join();
    move_worker_ = std::thread(&EdgeDaemon::ManualMoveWorker, this, target_yaw, target_pitch, false);
    return GetSnapshot();
  }

  if (cmd == "start_scan") {
    std::thread old_worker;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before starting a new one.";
        return state_;
      }
      // B8: a manual jog/home runs on the move worker; don't let a scan start
      // driving motion while one is in flight.
      if (manual_move_active_) {
        error_message = "Wait for the current move to finish before scanning.";
        return state_;
      }
      // B6: a scan is a long sequence of planned moves; refuse to start one from
      // an untrusted position. The operator must re-home first.
      if (!motion_->yaw_position_known() || !motion_->pitch_position_known()) {
        error_message = "Position unknown after an aborted move. Home the gantry before scanning.";
        return state_;
      }
      // scan_state_ == Idle implies the worker (if any) has already exited;
      // reap it before launching a fresh one.
      old_worker = std::move(scan_worker_);
      ResetScanLocked();
      scan_state_ = ScanState::Scanning;
      state_.mode = "scanning";
      AddLogLocked("scanner", "info", "Scan started.");
    }
    if (old_worker.joinable()) old_worker.join();
    scan_worker_ = std::thread(&EdgeDaemon::ScanWorker, this);
    return GetSnapshot();
  }

  if (cmd == "pause_scan") {
    std::scoped_lock lock(mutex_);
    if (scan_state_ != ScanState::Scanning) {
      error_message = "No scan in progress.";
      return state_;
    }
    // Cooperative pause: the worker finishes the current cell, then parks on
    // the condition variable until resume_scan. Motion is NOT aborted, so
    // position tracking stays consistent across pause/resume.
    scan_state_ = ScanState::Paused;
    state_.mode = "paused";
    AddLogLocked("scanner", "warn", "Scan paused.");
    return state_;
  }

  if (cmd == "resume_scan") {
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Paused) {
        error_message = "Scan is not paused.";
        return state_;
      }
      scan_state_ = ScanState::Scanning;
      state_.mode = "scanning";
      AddLogLocked("scanner", "info", "Scan resumed.");
    }
    scan_cv_.notify_all();
    return GetSnapshot();
  }

  if (cmd == "stop_scan") {
    std::thread prev;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ == ScanState::Idle) {
        error_message = "No scan in progress.";
        return state_;
      }
      scan_state_ = ScanState::Stopping;
      state_.mode = "idle";
      prev = std::move(scan_worker_);
      AddLogLocked("scanner", "warn", "Scan stopped.");
    }
    motion_->AbortMotion();
    scan_cv_.notify_all();
    if (prev.joinable()) prev.join();
    return GetSnapshot();
  }

  error_message = "Unsupported command: " + cmd;
  std::scoped_lock lock(mutex_);
  return state_;
}

// B8: runs a single home/jog off the HTTP thread. The blocking MotionController
// call (which drives the DMA waveform to completion or abort) happens here, not
// on the request thread, so /api/state polling — and thus the safety heartbeat
// — stays responsive throughout a multi-second move.
void EdgeDaemon::ManualMoveWorker(double yaw_deg, double pitch_deg, bool is_home) {
  const auto result = is_home ? motion_->Home() : motion_->MoveTo(yaw_deg, pitch_deg);

  std::scoped_lock lock(mutex_);
  manual_move_active_ = false;
  // If an estop/fault landed while we were moving, it owns the state — don't
  // clobber the "fault" mode or re-latch.
  if (safety_->faulted()) return;
  if (!result.success) {
    FailLocked(std::string(is_home ? "Home" : "Jog") + " failed: " + result.error);
  } else if (state_.mode == "manual") {
    state_.mode = "idle";
  }
}

void EdgeDaemon::ScanWorker() {
  if (config_.scan.mode == "sweep") {
    ScanWorkerSweep();
    return;
  }
  while (true) {
    int index = 0;
    int width = 0;
    int height = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      scan_cv_.wait(lock, [&] {
        return scan_state_ == ScanState::Scanning || scan_state_ == ScanState::Stopping;
      });
      if (scan_state_ == ScanState::Stopping) return;

      width  = grid_w_;
      height = grid_h_;
      if (width == 0 || height == 0) {
        scan_state_ = ScanState::Idle;
        state_.mode = "idle";
        return;
      }
      index = scan_index_;
      if (index >= width * height) {
        FinishScanLocked("Scan complete. Surface model updated.");
        scan_state_ = ScanState::Idle;
        return;
      }
    }

    const auto [x, y] = CoordForIndex(index, width);
    const double tgt_yaw   = TargetYawForCell(x, width);
    const double tgt_pitch = TargetPitchForCell(y, height);

    const auto move = motion_->MoveTo(tgt_yaw, tgt_pitch);
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ == ScanState::Stopping) return;
      if (!move.success) {
        FailLocked("Scan move failed: " + move.error);
        safety_->TriggerFault(FaultCode::MotionAbort, move.error);
        scan_state_ = ScanState::Idle;
        return;
      }
      if (scan_state_ == ScanState::Paused) continue;  // don't capture; re-park on CV
    }

    // B11: a single noisy I²C read shouldn't discard the whole scan. Retry a
    // few times (re-triggering and letting the bus settle between attempts)
    // before latching a LidarFault. Sweep mode already tolerates dropped reads
    // by skipping the cell; step mode used to latch on the very first NaN.
    constexpr int kLidarReadAttempts = 3;
    double distance = 0.0;
    bool got_reading = false;
    for (int attempt = 0; attempt < kLidarReadAttempts; ++attempt) {
      if (attempt > 0) {
        {
          std::scoped_lock lock(mutex_);
          if (scan_state_ == ScanState::Stopping) return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // let I²C settle
      }
      gpio_->PulseTrigger(static_cast<std::uint32_t>(config_.safety.lidar_trigger_pulse_us));
      distance = lidar_->ReadDistanceMeters(tgt_yaw, tgt_pitch);
      if (!std::isnan(distance)) { got_reading = true; break; }
    }

    std::scoped_lock lock(mutex_);
    if (scan_state_ == ScanState::Stopping) return;

    if (!got_reading) {
      FailLocked("Lidar read failed after " + std::to_string(kLidarReadAttempts) +
                 " attempts: " + lidar_->last_error());
      safety_->TriggerFault(FaultCode::LidarFault, lidar_->last_error());
      scan_state_ = ScanState::Idle;
      return;
    }

    // Normalise to a 0..1 "surface confidence" band for the web UI heatmap.
    const double normalised = Clamp((6.4 - distance) / 2.8, 0.0, 1.0);
    MarkCellLocked(x, y, Round(normalised, 10000.0));
    filled_cells_ = std::max(filled_cells_, index + 1);
    scan_index_ = index + 1;

    const int total = width * height;
    state_.coverage = Round(static_cast<double>(filled_cells_) / std::max(1, total), 10000.0);
    state_.scan_progress = state_.coverage;
    UpdateMetricsLocked();

    if (scan_index_ >= total) {
      FinishScanLocked("Scan complete. Surface model updated.");
      scan_state_ = ScanState::Idle;
      return;
    }
  }
}

void EdgeDaemon::ScanWorkerSweep() {
  // Continuous scan: sweep yaw across each row at a constant, LIDAR-limited
  // velocity while reading the sensor on the fly. The head only accelerates
  // and decelerates once per row (at the ends), not once per cell — which is
  // where stop-and-shoot scanning burns nearly all its time. Boustrophedon:
  // even rows sweep min→max, odd rows max→min, so no inter-row repositioning.
  const double mspd_yaw = config_.mechanics.yaw_microsteps_per_deg();

  while (true) {
    int width = 0, height = 0, row = 0;
    double yaw_min = 0.0, yaw_max = 0.0;
    bool forward = true;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      scan_cv_.wait(lock, [&] {
        return scan_state_ == ScanState::Scanning || scan_state_ == ScanState::Stopping;
      });
      if (scan_state_ == ScanState::Stopping) return;

      width  = grid_w_;
      height = grid_h_;
      if (width == 0 || height == 0) {
        scan_state_ = ScanState::Idle;
        state_.mode = "idle";
        return;
      }
      row = scan_row_;
      if (row >= height) {
        FinishScanLocked("Scan complete. Surface model updated.");
        scan_state_ = ScanState::Idle;
        return;
      }
      yaw_min = state_.scan_settings.yaw_min;
      yaw_max = state_.scan_settings.yaw_max;
      forward = (row % 2 == 0);
    }

    const double tgt_pitch = TargetPitchForCell(row, height);
    const double yaw_start = forward ? yaw_min : yaw_max;
    const double yaw_end   = forward ? yaw_max : yaw_min;

    // 1. Position the head at the row start + correct pitch (point-to-point).
    const auto pre = motion_->MoveTo(yaw_start, tgt_pitch);
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ == ScanState::Stopping) return;
      if (!pre.success) {
        FailLocked("Scan positioning failed: " + pre.error);
        safety_->TriggerFault(FaultCode::MotionAbort, pre.error);
        scan_state_ = ScanState::Idle;
        return;
      }
      if (scan_state_ == ScanState::Paused) continue;  // row not started; re-park on CV
    }

    // 2. LIDAR-limited sweep velocity: head advances at most ~one cell per
    //    LIDAR sample period, capped by the motor's sweep ceiling.
    const double cell_w = (yaw_max - yaw_min) / std::max(1, width - 1);
    const double lidar_period_s = std::max(1, config_.scan.lidar_period_ms) / 1000.0;
    const double v_sweep = std::min(config_.scan.sweep_max_speed_deg_s, cell_w / lidar_period_s);

    // 3. Launch the continuous sweep.
    std::string err;
    if (!motion_->StartYawSweep(yaw_end, v_sweep, config_.scan.sweep_accel_deg_s2, err)) {
      std::scoped_lock lock(mutex_);
      FailLocked("Sweep start failed: " + err);
      safety_->TriggerFault(FaultCode::MotionAbort, err);
      scan_state_ = ScanState::Idle;
      return;
    }

    // 4. Sample on the fly, binning each reading to a cell by head position.
    bool aborted = false;
    const double span = std::max(1e-9, yaw_max - yaw_min);
    while (motion_->SweepBusy()) {
      {
        std::scoped_lock lock(mutex_);
        if (scan_state_ == ScanState::Stopping) aborted = true;
      }
      if (aborted) { motion_->AbortMotion(); break; }

      const long s0 = motion_->SweepMicrostepsTravelled();
      const double yaw_pre = forward ? (yaw_start + s0 / mspd_yaw) : (yaw_start - s0 / mspd_yaw);
      gpio_->PulseTrigger(static_cast<std::uint32_t>(config_.safety.lidar_trigger_pulse_us));
      const double distance = lidar_->ReadDistanceMeters(yaw_pre, tgt_pitch);
      const long s1 = motion_->SweepMicrostepsTravelled();
      if (std::isnan(distance)) continue;  // transient bad read: skip, keep sweeping

      const double trav_deg = (static_cast<double>(s0 + s1) / 2.0) / mspd_yaw;
      const double yaw_now = forward ? (yaw_start + trav_deg) : (yaw_start - trav_deg);
      int x = static_cast<int>(std::lround((yaw_now - yaw_min) / span * (width - 1)));
      if (x < 0) x = 0;
      if (x >= width) x = width - 1;

      const double normalised = Clamp((6.4 - distance) / 2.8, 0.0, 1.0);
      std::scoped_lock lock(mutex_);
      if (scan_state_ == ScanState::Stopping) { aborted = true; }
      else { MarkCellLocked(x, row, Round(normalised, 10000.0)); }
      if (aborted) { motion_->AbortMotion(); break; }
    }

    motion_->FinishYawSweep(!aborted);

    std::scoped_lock lock(mutex_);
    if (scan_state_ == ScanState::Stopping) return;
    scan_row_ = row + 1;
    filled_cells_ = scan_row_ * width;  // rows completed (per-cell gaps are cosmetic)
    const int total = width * height;
    state_.coverage = Round(static_cast<double>(filled_cells_) / std::max(1, total), 10000.0);
    state_.scan_progress = state_.coverage;
    UpdateMetricsLocked();
    if (scan_row_ >= height) {
      FinishScanLocked("Scan complete. Surface model updated.");
      scan_state_ = ScanState::Idle;
      return;
    }
  }
}

std::pair<int, int> EdgeDaemon::CoordForIndex(int index, int width) const {
  int row = width > 0 ? index / width : 0;
  int col = width > 0 ? index % width : 0;
  if (row % 2 == 1) col = (width - 1) - col;  // boustrophedon: halves the seek distance
  return {col, row};
}

double EdgeDaemon::TargetYawForCell(int x, int width) const {
  const double range = state_.scan_settings.yaw_max - state_.scan_settings.yaw_min;
  const double denom = std::max(1, width - 1);
  return state_.scan_settings.yaw_min + (static_cast<double>(x) / denom) * range;
}

double EdgeDaemon::TargetPitchForCell(int y, int height) const {
  const double range = state_.scan_settings.pitch_max - state_.scan_settings.pitch_min;
  const double denom = std::max(1, height - 1);
  return state_.scan_settings.pitch_min + (static_cast<double>(y) / denom) * range;
}

void EdgeDaemon::ApplyResolutionLocked(const std::string& resolution) {
  // Memory/transmission backstop: a per-microstep scan over the full envelope
  // is tens of millions of cells. Clamp the stored grid to this and tell the
  // operator to narrow the scan range for genuine per-microstep detail.
  constexpr long kMaxScanCells = 300000;

  const std::string preset = resolution.empty() ? state_.scan_settings.resolution : resolution;
  const int ms = std::max(1, config_.mechanics.microsteps);

  // Resolution presets are grounded in hardware: the scan head advances a
  // fixed number of microsteps between samples. "max" samples every single
  // microstep (the finest the driver can resolve); coarser presets stride by
  // whole motor steps.
  int stride;
  if      (preset == "max")    stride = 1;
  else if (preset == "fine")   stride = std::max(1, ms / 8);
  else if (preset == "coarse") stride = std::max(1, ms * 4);
  else                         stride = ms;  // "standard" (and anything else) = 1 full step

  // Grid dimensions fall out of the scan range, microstep density and stride.
  const double yaw_span   = std::max(0.0, state_.scan_settings.yaw_max   - state_.scan_settings.yaw_min);
  const double pitch_span = std::max(0.0, state_.scan_settings.pitch_max - state_.scan_settings.pitch_min);
  long width  = static_cast<long>(yaw_span   * config_.mechanics.yaw_microsteps_per_deg()   / stride) + 1;
  long height = static_cast<long>(pitch_span * config_.mechanics.pitch_microsteps_per_deg() / stride) + 1;
  width  = std::max(2L, width);
  height = std::max(2L, height);

  if (width * height > kMaxScanCells) {
    const double scale = std::sqrt(static_cast<double>(kMaxScanCells) /
                                   static_cast<double>(width * height));
    width  = std::max(2L, static_cast<long>(static_cast<double>(width)  * scale));
    height = std::max(2L, static_cast<long>(static_cast<double>(height) * scale));
    AddLogLocked("scanner", "warn",
                 "Requested density exceeds the " + std::to_string(kMaxScanCells) +
                 "-cell limit; grid clamped — narrow the scan range for finer detail.");
  }

  state_.scan_settings.resolution = preset;
  state_.scan_settings.sample_stride_microsteps = stride;
  RebuildGridLocked(static_cast<int>(width), static_cast<int>(height));

  double duration_s;
  if (config_.scan.mode == "sweep") {
    // Continuous: per-row time ≈ sweep duration + a fixed ramp/pitch-step budget.
    const double cell_w = yaw_span / static_cast<double>(std::max(1L, width - 1));
    const double lidar_period_s = std::max(1, config_.scan.lidar_period_ms) / 1000.0;
    const double v = std::min(config_.scan.sweep_max_speed_deg_s, cell_w / lidar_period_s);
    const double row_s = (v > 0.0 ? yaw_span / v : 0.0) + 0.4;
    duration_s = row_s * static_cast<double>(height);
  } else {
    const double per_point_ms = 110.0;  // move + settle + trigger + read
    duration_s = (static_cast<double>(width) * static_cast<double>(height) * per_point_ms) / 1000.0;
  }
  state_.scan_duration_seconds = Round(duration_s, 100.0);
  ResetScanLocked();
}

void EdgeDaemon::ResetScanLocked() {
  ClearGridLocked();  // wipe cells + bump generation so clients full-refresh to empty
  state_.coverage = 0.0;
  state_.scan_progress = 0.0;
  state_.last_completed_scan_at.reset();
  filled_cells_ = 0;
  scan_index_ = 0;
  scan_row_ = 0;
}

void EdgeDaemon::RebuildGridLocked(int width, int height) {
  grid_w_ = std::max(0, width);
  grid_h_ = std::max(0, height);
  grid_.assign(static_cast<std::size_t>(grid_h_),
               std::vector<double>(static_cast<std::size_t>(grid_w_), -1.0));
  cell_version_.assign(static_cast<std::size_t>(grid_w_) * static_cast<std::size_t>(grid_h_), 0);
  ++grid_generation_;
  grid_version_ = 0;
}

void EdgeDaemon::ClearGridLocked() {
  for (auto& row : grid_) std::fill(row.begin(), row.end(), -1.0);
  std::fill(cell_version_.begin(), cell_version_.end(), std::uint64_t{0});
  ++grid_generation_;
  grid_version_ = 0;
}

void EdgeDaemon::MarkCellLocked(int x, int y, double value) {
  if (x < 0 || y < 0 || x >= grid_w_ || y >= grid_h_) return;
  grid_[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] = value;
  cell_version_[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid_w_) +
                static_cast<std::size_t>(x)] = ++grid_version_;
}

GridUpdate EdgeDaemon::GetGridUpdate(std::uint64_t since_version,
                                     std::uint64_t client_generation) const {
  std::scoped_lock lock(mutex_);
  GridUpdate gu;
  gu.generation = grid_generation_;
  gu.version = grid_version_;
  gu.width = grid_w_;
  gu.height = grid_h_;
  // A stale client generation (fresh client, or post-resize/reset) ⇒ send every
  // filled cell so it can rebuild; otherwise only cells changed since its version.
  gu.full = (client_generation != grid_generation_);
  for (int y = 0; y < grid_h_; ++y) {
    for (int x = 0; x < grid_w_; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(grid_w_) +
                            static_cast<std::size_t>(x);
      const double v = grid_[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
      if (v < 0.0) continue;  // unfilled cells aren't transmitted; clients default to -1
      if (gu.full || cell_version_[i] > since_version) {
        gu.idx.push_back(static_cast<int>(i));
        gu.val.push_back(v);
      }
    }
  }
  return gu;
}

void EdgeDaemon::AddLogLocked(const std::string& source, const std::string& level,
                              const std::string& message) {
  state_.activity.insert(state_.activity.begin(),
                         ActivityEntry{source, CurrentTimestamp(), message, level});
  if (state_.activity.size() > 20) state_.activity.resize(20);
}

void EdgeDaemon::UpdateMetricsLocked() {
  const bool scanning = state_.mode == "scanning";
  const bool moving   = motion_->is_busy();
  state_.metrics.motor_temp_c   = Round(31.0 + (scanning ? 4.0 : 0.8) + (moving ? 1.2 : 0.0), 10.0);
  state_.metrics.motor_current_a = Round(moving ? 1.45 : 0.42, 100.0);
  state_.metrics.lidar_fps      = scanning ? 12 : 0;
  state_.metrics.radar_fps      = 0;
  state_.metrics.latency_ms     = scanning ? 80 : 30;
}

void EdgeDaemon::FinishScanLocked(const std::string& message) {
  state_.mode = "idle";
  state_.coverage = 1.0;
  state_.scan_progress = 1.0;
  state_.last_completed_scan_at = CurrentTimestamp();
  AddLogLocked("scanner", "info", message);
}

void EdgeDaemon::FailLocked(const std::string& message) {
  state_.connected = false;
  state_.mode = "fault";
  state_.faults = {message};
  AddLogLocked("hardware", "error", message);
}

}  // namespace edge
