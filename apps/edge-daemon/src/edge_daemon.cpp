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
  const std::time_t now_time = clock::to_time_t(now);
  std::tm tm_utc{};
#ifdef _WIN32
  gmtime_s(&tm_utc, &now_time);
#else
  gmtime_r(&now_time, &tm_utc);
#endif

  std::ostringstream out;
  out << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

std::pair<int, int> CoordForIndex(int index, int width) {
  const int row = width > 0 ? index / width : 0;
  int col = width > 0 ? index % width : 0;
  if (row % 2 == 1) {
    col = (width - 1) - col;
  }
  return {col, row};
}

}  // namespace

EdgeDaemon::EdgeDaemon(Config config) : config_(std::move(config)) {
  state_.scan_settings.resolution = "medium";
  state_.scan_duration_seconds = ScanDurationForResolution(state_.scan_settings.resolution);
  state_.grid.assign(config_.grid_height, std::vector<double>(config_.grid_width, -1.0));
  serial_ = std::make_unique<SerialTransport>(config_.serial_port, config_.serial_baud);
  lidar_ = CreateLidarSensor(config_);

  AddLogLocked("system", "info", "Edge daemon initialized.");
  AddLogLocked("sensor", "info", std::string("Lidar source set to ") + lidar_->Name() + ".");
}

EdgeDaemon::~EdgeDaemon() { Stop(); }

bool EdgeDaemon::Start() {
  std::scoped_lock lock(mutex_);
  if (running_) {
    return true;
  }

  if (!lidar_->Initialize()) {
    state_.faults = {"Failed to initialize lidar sensor."};
    state_.connected = false;
    AddLogLocked("sensor", "error", "Lidar initialization failed.");
  }

  if (!config_.simulate_hardware && config_.enable_serial) {
    if (!serial_->Open()) {
      state_.connected = false;
      state_.faults = {"Unable to open Arduino serial port: " + serial_->last_error()};
      AddLogLocked("motion", "error", "Arduino serial link unavailable.");
    } else {
      state_.connected = true;
      AddLogLocked("motion", "info", "Arduino serial link opened.");
    }
  } else {
    state_.connected = true;
    AddLogLocked("motion", "info", "Running with simulated motor transport.");
  }

  running_ = true;
  worker_ = std::thread(&EdgeDaemon::RunLoop, this);
  return true;
}

void EdgeDaemon::Stop() {
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
  if (serial_) {
    serial_->Close();
  }
}

Snapshot EdgeDaemon::GetSnapshot() const {
  std::scoped_lock lock(mutex_);
  return state_;
}

Snapshot EdgeDaemon::ExecuteCommand(const CommandRequest& request, std::string& error_message) {
  std::scoped_lock lock(mutex_);

  const std::string command = request.command;
  error_message.clear();

  if (command == "connect") {
    if (!config_.simulate_hardware && config_.enable_serial && !serial_->IsOpen()) {
      if (!serial_->Open()) {
        error_message = "Unable to open Arduino serial port: " + serial_->last_error();
        state_.connected = false;
        state_.faults = {error_message};
        return state_;
      }
    }
    state_.connected = true;
    state_.faults.clear();
    AddLogLocked("motion", "info", "Hardware transport connected.");
    return state_;
  }

  if (!state_.faults.empty() && command != "clear_fault" && command != "estop") {
    error_message = "Clear the fault before issuing this command.";
    return state_;
  }

  if (!SendPrototypeCommandLocked(request, error_message)) {
    return state_;
  }

  if (command == "home") {
    state_.mode = "idle";
    state_.yaw = 0.0;
    state_.pitch = 0.0;
    AddLogLocked("motion", "info", "Gantry returned to home position.");
  } else if (command == "jog") {
    if (request.axis == "yaw") {
      state_.yaw = Round(Clamp(state_.yaw + request.delta, state_.scan_settings.yaw_min, state_.scan_settings.yaw_max), 100.0);
    } else if (request.axis == "pitch") {
      state_.pitch = Round(Clamp(state_.pitch + request.delta, state_.scan_settings.pitch_min, state_.scan_settings.pitch_max), 100.0);
    }
    state_.mode = "manual";
    AddLogLocked("motion", "info", "Jogged " + request.axis + ".");
  } else if (command == "set_resolution") {
    state_.scan_settings.resolution = request.resolution;
    state_.scan_duration_seconds = ScanDurationForResolution(request.resolution);
    AddLogLocked("scanner", "info", "Scan resolution set to " + request.resolution + ".");
  } else if (command == "start_scan") {
    ResetScanLocked();
    state_.mode = "scanning";
    scan_started_at_ = std::chrono::steady_clock::now();
    AddLogLocked("scanner", "info", "Scan started.");
  } else if (command == "pause_scan") {
    if (state_.mode == "scanning") {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - scan_started_at_).count();
      scan_accumulated_seconds_ += elapsed;
      state_.mode = "paused";
      AddLogLocked("scanner", "warn", "Scan paused.");
    }
  } else if (command == "resume_scan") {
    state_.mode = "scanning";
    scan_started_at_ = std::chrono::steady_clock::now();
    AddLogLocked("scanner", "info", "Scan resumed.");
  } else if (command == "stop_scan") {
    state_.mode = "idle";
    scan_accumulated_seconds_ = 0.0;
    AddLogLocked("scanner", "warn", "Scan stopped.");
  } else if (command == "estop") {
    state_.mode = "fault";
    state_.faults = {"Emergency stop asserted"};
    scan_accumulated_seconds_ = 0.0;
    AddLogLocked("safety", "error", "Emergency stop triggered.");
  } else if (command == "clear_fault") {
    state_.faults.clear();
    state_.mode = "idle";
    AddLogLocked("safety", "info", "Fault state cleared.");
  }

  return state_;
}

bool EdgeDaemon::Healthy() const { return running_.load(); }

void EdgeDaemon::RunLoop() {
  while (running_) {
    Tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.tick_interval_ms));
  }
}

void EdgeDaemon::Tick() {
  std::scoped_lock lock(mutex_);
  state_.scan_duration_seconds = ScanDurationForResolution(state_.scan_settings.resolution);

  if (state_.mode == "scanning") {
    const double elapsed = scan_accumulated_seconds_ +
                           std::chrono::duration<double>(std::chrono::steady_clock::now() - scan_started_at_).count();
    const double progress = Clamp(elapsed / std::max(1.0, state_.scan_duration_seconds), 0.0, 1.0);
    const int total_cells = config_.grid_width * config_.grid_height;
    const int target = static_cast<int>(std::floor(progress * total_cells));
    FillToLocked(target);
    state_.scan_progress = Round(progress, 10000.0);
    SetHeadLocked(filled_cells_);

    if (progress >= 1.0) {
      state_.mode = "idle";
      state_.last_completed_scan_at = CurrentTimestamp();
      scan_accumulated_seconds_ = 0.0;
      AddLogLocked("scanner", "info", "Scan complete. Surface model updated.");
    }
  } else if (state_.mode == "paused") {
    state_.scan_progress = Round(Clamp(scan_accumulated_seconds_ / std::max(1.0, state_.scan_duration_seconds), 0.0, 1.0), 10000.0);
  }

  UpdateMetricsLocked();
}

void EdgeDaemon::ResetScanLocked() {
  state_.grid.assign(config_.grid_height, std::vector<double>(config_.grid_width, -1.0));
  state_.coverage = 0.0;
  state_.scan_progress = 0.0;
  state_.last_completed_scan_at.reset();
  filled_cells_ = 0;
  scan_accumulated_seconds_ = 0.0;
}

void EdgeDaemon::FillToLocked(int target_filled) {
  const int max_cells = config_.grid_width * config_.grid_height;
  target_filled = std::clamp(target_filled, 0, max_cells);

  for (int index = filled_cells_; index < target_filled; ++index) {
    const auto [x, y] = CoordForIndex(index, config_.grid_width);
    const double yaw_range = state_.scan_settings.yaw_max - state_.scan_settings.yaw_min;
    const double pitch_range = state_.scan_settings.pitch_max - state_.scan_settings.pitch_min;
    const double yaw = state_.scan_settings.yaw_min +
                       (static_cast<double>(x) / std::max(1, config_.grid_width - 1)) * yaw_range;
    const double pitch = state_.scan_settings.pitch_min +
                         (static_cast<double>(y) / std::max(1, config_.grid_height - 1)) * pitch_range;
    const double distance = lidar_->ReadDistanceMeters(yaw, pitch);
    const double normalized = Clamp((6.4 - distance) / 2.8, 0.0, 1.0);
    state_.grid[y][x] = Round(normalized, 10000.0);
  }

  filled_cells_ = target_filled;
  state_.coverage = Round(static_cast<double>(filled_cells_) / std::max(1, max_cells), 10000.0);
}

void EdgeDaemon::SetHeadLocked(int index) {
  const int max_index = std::max(0, (config_.grid_width * config_.grid_height) - 1);
  if (index > max_index) {
    index = max_index;
  }

  const auto [x, y] = CoordForIndex(index, config_.grid_width);
  const double yaw_range = state_.scan_settings.yaw_max - state_.scan_settings.yaw_min;
  const double pitch_range = state_.scan_settings.pitch_max - state_.scan_settings.pitch_min;
  state_.yaw = Round(state_.scan_settings.yaw_min + (static_cast<double>(x) / std::max(1, config_.grid_width - 1)) * yaw_range, 100.0);
  state_.pitch = Round(state_.scan_settings.pitch_min + (static_cast<double>(y) / std::max(1, config_.grid_height - 1)) * pitch_range, 100.0);
}

void EdgeDaemon::UpdateMetricsLocked() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const double phase = std::chrono::duration<double>(now).count();
  const double scan_load = state_.mode == "scanning" ? 1.0 : 0.0;

  state_.metrics.motor_temp_c = Round(31.0 + (std::sin(phase * 0.35) * 1.8) + (scan_load * 3.0), 10.0);
  state_.metrics.motor_current_a = Round(1.2 + (scan_load * 0.45) + ((1.0 - scan_load) * 0.08) + (std::cos(phase * 0.52) * 0.08), 100.0);
  state_.metrics.lidar_fps = static_cast<int>(std::round(18 + (scan_load * 6.0) + (std::sin(phase) * 1.5)));
  state_.metrics.radar_fps = 0;
  state_.metrics.latency_ms = static_cast<int>(std::round(36 + (scan_load * 14.0) + ((1.0 - scan_load) * 4.0) + (std::abs(std::sin(phase * 0.45)) * 8)));
}

void EdgeDaemon::AddLogLocked(const std::string& source, const std::string& level, const std::string& message) {
  state_.activity.insert(state_.activity.begin(), ActivityEntry{source, CurrentTimestamp(), message, level});
  if (state_.activity.size() > 20) {
    state_.activity.resize(20);
  }
}

bool EdgeDaemon::SendPrototypeCommandLocked(const CommandRequest& request, std::string& error_message) {
  if (config_.simulate_hardware || !config_.enable_serial) {
    state_.connected = true;
    return true;
  }

  if (!serial_->IsOpen()) {
    error_message = "Arduino serial port is not connected.";
    state_.connected = false;
    return false;
  }

  auto send_ascii = [&](const std::string& line) -> bool {
    if (!serial_->SendLine(line)) {
      error_message = "Failed to write to Arduino: " + serial_->last_error();
      state_.connected = false;
      return false;
    }
    state_.connected = true;
    return true;
  };

  if (request.command == "home") return send_ascii("HOME");
  if (request.command == "start_scan") return send_ascii("START");
  if (request.command == "pause_scan") return send_ascii("PAUSE");
  if (request.command == "resume_scan") return send_ascii("START");
  if (request.command == "stop_scan") return send_ascii("STOP");
  if (request.command == "estop") return send_ascii("ESTOP");
  if (request.command == "clear_fault") return send_ascii("CLEAR_FAULT");
  if (request.command == "set_resolution") return true;

  if (request.command != "jog") {
    error_message = "Unsupported command.";
    return false;
  }

  if (request.axis == "yaw") {
    const std::string line = request.delta >= 0 ? "JOG_YAW_POS" : "JOG_YAW_NEG";
    const int repeats = std::max(1, static_cast<int>(std::round(std::abs(request.delta) / 5.0)));
    for (int i = 0; i < repeats; ++i) {
      if (!send_ascii(line)) return false;
    }
    return true;
  }

  if (request.axis == "pitch") {
    const std::string line = request.delta >= 0 ? "JOG_PITCH_POS" : "JOG_PITCH_NEG";
    const int repeats = std::max(1, static_cast<int>(std::round(std::abs(request.delta) / 3.0)));
    for (int i = 0; i < repeats; ++i) {
      if (!send_ascii(line)) return false;
    }
    return true;
  }

  error_message = "Unsupported jog axis.";
  return false;
}

double EdgeDaemon::ScanDurationForResolution(const std::string& resolution) {
  if (resolution == "low") return 20.0;
  if (resolution == "high") return 56.0;
  return 36.0;
}

double EdgeDaemon::Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

double EdgeDaemon::Round(double value, double scale) {
  return std::round(value * scale) / scale;
}

}  // namespace edge
