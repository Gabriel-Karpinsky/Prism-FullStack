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
  if (safety_) safety_->Stop();
  if (motion_ && was_running) motion_->SetEnabled(false);
  if (gpio_   && was_running) gpio_->Shutdown();
}

bool EdgeDaemon::Healthy() const {
  return running_.load() && safety_ && !safety_->faulted();
}

Snapshot EdgeDaemon::GetSnapshot() const {
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
    std::thread prev;
    {
      std::scoped_lock lock(mutex_);
      state_.mode = "fault";
      if (state_.faults.empty()) state_.faults = {"Emergency stop asserted."};
      if (scan_state_ != ScanState::Idle) {
        scan_state_ = ScanState::Stopping;
        prev = std::move(scan_worker_);
      }
      AddLogLocked("safety", "error", "Emergency stop triggered.");
    }
    scan_cv_.notify_all();
    if (prev.joinable()) prev.join();
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
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before homing.";
        return state_;
      }
      state_.mode = "manual";
      AddLogLocked("motion", "info", "Homing to (0, 0).");
    }
    const auto result = motion_->Home();
    std::scoped_lock lock(mutex_);
    if (!result.success) {
      error_message = "Home failed: " + result.error;
      FailLocked(error_message);
    } else {
      state_.mode = "idle";
    }
    return state_;
  }

  if (cmd == "jog") {
    if (request.axis != "yaw" && request.axis != "pitch") {
      error_message = "Unsupported jog axis.";
      std::scoped_lock lock(mutex_);
      return state_;
    }
    double target_yaw;
    double target_pitch;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before jogging.";
        return state_;
      }
      target_yaw   = motion_->yaw_deg();
      target_pitch = motion_->pitch_deg();
      if (request.axis == "yaw") target_yaw   += request.delta;
      else                       target_pitch += request.delta;
      state_.mode = "manual";
      AddLogLocked("motion", "info", std::string("Jog ") + request.axis + ".");
    }
    const auto result = motion_->MoveTo(target_yaw, target_pitch);
    std::scoped_lock lock(mutex_);
    if (!result.success) {
      error_message = "Jog failed: " + result.error;
      FailLocked(error_message);
    } else {
      state_.mode = "idle";
    }
    return state_;
  }

  if (cmd == "start_scan") {
    std::thread old_worker;
    {
      std::scoped_lock lock(mutex_);
      if (scan_state_ != ScanState::Idle) {
        error_message = "Stop the current scan before starting a new one.";
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

void EdgeDaemon::ScanWorker() {
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

      width  = state_.grid.empty() ? 0 : static_cast<int>(state_.grid.front().size());
      height = static_cast<int>(state_.grid.size());
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

    gpio_->PulseTrigger(static_cast<std::uint32_t>(config_.safety.lidar_trigger_pulse_us));
    const double distance = lidar_->ReadDistanceMeters(tgt_yaw, tgt_pitch);

    std::scoped_lock lock(mutex_);
    if (scan_state_ == ScanState::Stopping) return;

    if (std::isnan(distance)) {
      FailLocked("Lidar read failed: " + lidar_->last_error());
      safety_->TriggerFault(FaultCode::LidarFault, lidar_->last_error());
      scan_state_ = ScanState::Idle;
      return;
    }

    // Normalise to a 0..1 "surface confidence" band for the web UI heatmap.
    const double normalised = Clamp((6.4 - distance) / 2.8, 0.0, 1.0);
    state_.grid[y][x] = Round(normalised, 10000.0);
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
  int width  = config_.service.grid_width;
  int height = config_.service.grid_height;
  if (resolution == "low") {
    width = 24; height = 12;
  } else if (resolution == "high") {
    width  = std::max(72, config_.service.grid_width  + config_.service.grid_width  / 2);
    height = std::max(36, config_.service.grid_height + config_.service.grid_height / 2);
  }

  state_.scan_settings.resolution = resolution.empty() ? std::string("medium") : resolution;
  state_.grid.assign(height, std::vector<double>(width, -1.0));
  const double per_point_ms = 80.0;  // rough budget: move + settle + trigger + read
  state_.scan_duration_seconds =
      Round((static_cast<double>(width) * height * per_point_ms) / 1000.0, 100.0);
  ResetScanLocked();
}

void EdgeDaemon::ResetScanLocked() {
  for (auto& row : state_.grid) std::fill(row.begin(), row.end(), -1.0);
  state_.coverage = 0.0;
  state_.scan_progress = 0.0;
  state_.last_completed_scan_at.reset();
  filled_cells_ = 0;
  scan_index_ = 0;
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
