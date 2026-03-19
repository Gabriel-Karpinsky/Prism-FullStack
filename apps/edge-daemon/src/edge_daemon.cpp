#include "edge_daemon.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
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

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::string FormatCommandFloat(double value) {
  std::ostringstream out;
  out.setf(std::ios::fixed, std::ios::floatfield);
  out.precision(2);
  out << value;
  return out.str();
}

}  // namespace

EdgeDaemon::EdgeDaemon(Config config) : config_(std::move(config)) {
  serial_ = std::make_unique<SerialTransport>(config_.serial_port, config_.serial_baud);
  lidar_ = CreateLidarSensor(config_);
  ApplyResolutionLocked(state_.scan_settings.resolution);
  ResetScanLocked();

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
    state_.faults = {"Failed to initialize lidar sensor: " + lidar_->last_error()};
    AddLogLocked("sensor", "error", state_.faults.front());
  }

  if (!config_.simulate_hardware && config_.enable_serial) {
    if (!serial_->Open()) {
      state_.connected = false;
      state_.faults = {"Unable to open Arduino serial port: " + serial_->last_error()};
      AddLogLocked("motion", "error", state_.faults.front());
    } else {
      state_.connected = true;
      AddLogLocked("motion", "info", "Arduino serial link opened.");
      RefreshHardwareStateLocked();
    }
  } else {
    state_.connected = true;
    AddLogLocked("motion", "info", "Running with simulated motion transport.");
  }

  const auto now = std::chrono::steady_clock::now();
  last_status_poll_at_ = now - std::chrono::milliseconds(config_.status_poll_interval_ms);
  last_heartbeat_at_ = now - std::chrono::milliseconds(config_.heartbeat_interval_ms);
  last_move_issued_at_ = now;
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
  error_message.clear();

  const std::string command = request.command;
  const bool using_simulation = config_.simulate_hardware || !config_.enable_serial;

  if (command == "connect") {
    if (!using_simulation && !serial_->IsOpen() && !serial_->Open()) {
      error_message = "Unable to open Arduino serial port: " + serial_->last_error();
      FailHardwareLocked(error_message);
      return state_;
    }

    state_.connected = true;
    if (!using_simulation && !RefreshHardwareStateLocked()) {
      error_message = "Unable to query Arduino status after connect.";
    }
    AddLogLocked("motion", "info", "Hardware transport connected.");
    return state_;
  }

  if (!state_.faults.empty() && command != "clear_fault" && command != "estop") {
    error_message = "Clear the fault before issuing this command.";
    return state_;
  }

  if (command == "set_resolution") {
    if (state_.mode == "scanning" || state_.mode == "paused") {
      error_message = "Stop the current scan before changing resolution.";
      return state_;
    }
    ApplyResolutionLocked(request.resolution.empty() ? state_.scan_settings.resolution : request.resolution);
    AddLogLocked("scanner", "info", "Scan resolution set to " + state_.scan_settings.resolution + ".");
    return state_;
  }

  std::string response;
  if (command == "home") {
    if (!SendArduinoCommandLocked("HOME", response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    state_.mode = "manual";
    if (using_simulation) {
      state_.yaw = 0.0;
      state_.pitch = 0.0;
    } else {
      RefreshHardwareStateLocked();
    }
    AddLogLocked("motion", "info", "Gantry moving to home position.");
    return state_;
  }

  if (command == "jog") {
    if (request.axis != "yaw" && request.axis != "pitch") {
      error_message = "Unsupported jog axis.";
      return state_;
    }
    const std::string line = "JOG " + request.axis + " " + FormatCommandFloat(request.delta);
    if (!SendArduinoCommandLocked(line, response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    state_.mode = "manual";
    if (using_simulation) {
      if (request.axis == "yaw") {
        state_.yaw = Clamp(state_.yaw + request.delta, state_.scan_settings.yaw_min, state_.scan_settings.yaw_max);
      } else {
        state_.pitch = Clamp(state_.pitch + request.delta, state_.scan_settings.pitch_min, state_.scan_settings.pitch_max);
      }
    } else {
      RefreshHardwareStateLocked();
    }
    AddLogLocked("motion", "info", "Jogged " + request.axis + ".");
    return state_;
  }

  if (command == "start_scan") {
    if (!SendArduinoCommandLocked("START_SCAN", response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    ResetScanLocked();
    state_.mode = "scanning";
    current_scan_index_ = 0;
    scan_waiting_for_settle_ = false;
    hardware_moving_ = false;
    AddLogLocked("scanner", "info", "Scan started.");
    return state_;
  }

  if (command == "pause_scan") {
    if (!SendArduinoCommandLocked("PAUSE_SCAN", response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    state_.mode = "paused";
    scan_waiting_for_settle_ = false;
    hardware_moving_ = false;
    AddLogLocked("scanner", "warn", "Scan paused.");
    return state_;
  }

  if (command == "resume_scan") {
    if (!SendArduinoCommandLocked("RESUME_SCAN", response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    state_.mode = "scanning";
    scan_waiting_for_settle_ = false;
    hardware_moving_ = false;
    AddLogLocked("scanner", "info", "Scan resumed.");
    return state_;
  }

  if (command == "stop_scan") {
    if (!SendArduinoCommandLocked("STOP_SCAN", response)) {
      error_message = response;
      FailHardwareLocked(error_message);
      return state_;
    }
    state_.mode = "idle";
    scan_waiting_for_settle_ = false;
    hardware_moving_ = false;
    AddLogLocked("scanner", "warn", "Scan stopped.");
    return state_;
  }

  if (command == "estop") {
    if (!SendArduinoCommandLocked("ESTOP", response)) {
      error_message = response;
    }
    state_.mode = "fault";
    state_.faults = {"Emergency stop asserted."};
    scan_waiting_for_settle_ = false;
    hardware_moving_ = false;
    AddLogLocked("safety", "error", "Emergency stop triggered.");
    return state_;
  }

  if (command == "clear_fault") {
    if (!SendArduinoCommandLocked("CLEAR_FAULT", response)) {
      error_message = response;
      return state_;
    }
    state_.faults.clear();
    state_.mode = "idle";
    hardware_moving_ = false;
    scan_waiting_for_settle_ = false;
    AddLogLocked("safety", "info", "Fault state cleared.");
    if (!using_simulation) {
      RefreshHardwareStateLocked();
    }
    return state_;
  }

  error_message = "Unsupported command.";
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
  const auto now = std::chrono::steady_clock::now();
  const bool using_simulation = config_.simulate_hardware || !config_.enable_serial;

  if (!using_simulation && serial_->IsOpen() &&
      now - last_status_poll_at_ >= std::chrono::milliseconds(config_.status_poll_interval_ms)) {
    last_status_poll_at_ = now;
    if (!RefreshHardwareStateLocked() && state_.mode == "scanning") {
      FailHardwareLocked("Failed to refresh Arduino status.");
    }
  }

  if (!using_simulation && serial_->IsOpen() &&
      (state_.mode == "manual" || state_.mode == "scanning" || state_.mode == "paused") &&
      now - last_heartbeat_at_ >= std::chrono::milliseconds(config_.heartbeat_interval_ms)) {
    last_heartbeat_at_ = now;
    std::string response;
    if (!SendArduinoCommandLocked("HEARTBEAT", response, false) && state_.mode == "scanning") {
      FailHardwareLocked("Failed to maintain Arduino heartbeat: " + response);
    }
  }

  if (state_.mode == "scanning" && state_.faults.empty()) {
    const int total_cells = static_cast<int>(state_.grid.size() * (state_.grid.empty() ? 0 : state_.grid.front().size()));
    if (current_scan_index_ >= total_cells) {
      FinishScanLocked("Scan complete. Surface model updated.");
    } else if (!scan_waiting_for_settle_) {
      std::string error_message;
      if (!IssueMoveToCellLocked(current_scan_index_, error_message)) {
        FailHardwareLocked(error_message);
      } else {
        scan_waiting_for_settle_ = true;
        last_move_issued_at_ = now;
      }
    } else {
      const bool settled = (using_simulation || !hardware_moving_) &&
                           now - last_move_issued_at_ >= std::chrono::milliseconds(config_.move_settle_ms);
      if (settled) {
        std::string error_message;
        if (!CaptureCurrentCellLocked(current_scan_index_, error_message)) {
          FailHardwareLocked(error_message);
        } else {
          ++current_scan_index_;
          scan_waiting_for_settle_ = false;
          if (current_scan_index_ >= total_cells) {
            FinishScanLocked("Scan complete. Surface model updated.");
          }
        }
      }
    }
  }

  UpdateMetricsLocked();
}

void EdgeDaemon::ResetScanLocked() {
  for (auto& row : state_.grid) {
    std::fill(row.begin(), row.end(), -1.0);
  }
  state_.coverage = 0.0;
  state_.scan_progress = 0.0;
  state_.last_completed_scan_at.reset();
  filled_cells_ = 0;
  current_scan_index_ = 0;
  scan_waiting_for_settle_ = false;
  hardware_moving_ = false;
}

void EdgeDaemon::ApplyResolutionLocked(const std::string& resolution) {
  int width = config_.grid_width;
  int height = config_.grid_height;
  if (resolution == "low") {
    width = 24;
    height = 12;
  } else if (resolution == "high") {
    width = std::max(72, config_.grid_width + (config_.grid_width / 2));
    height = std::max(36, config_.grid_height + (config_.grid_height / 2));
  }

  state_.scan_settings.resolution = resolution.empty() ? "medium" : resolution;
  state_.grid.assign(height, std::vector<double>(width, -1.0));
  state_.scan_duration_seconds = Round((static_cast<double>(width * height) * config_.estimated_point_time_ms) / 1000.0, 100.0);
  ResetScanLocked();
}

void EdgeDaemon::UpdateMetricsLocked() {
  const bool active_scan = state_.mode == "scanning";
  const bool active_motion = hardware_moving_ || scan_waiting_for_settle_ || state_.mode == "manual";

  state_.metrics.motor_temp_c = Round(31.0 + (active_scan ? 4.0 : 0.8) + (active_motion ? 1.2 : 0.0), 10.0);
  state_.metrics.motor_current_a = Round(active_motion ? 1.45 : 0.42, 100.0);
  state_.metrics.lidar_fps = active_scan ? std::max(1, 1000 / std::max(1, config_.estimated_point_time_ms)) : 0;
  state_.metrics.radar_fps = 0;
  state_.metrics.latency_ms = active_scan ? config_.estimated_point_time_ms : 30;
}

void EdgeDaemon::AddLogLocked(const std::string& source, const std::string& level, const std::string& message) {
  state_.activity.insert(state_.activity.begin(), ActivityEntry{source, CurrentTimestamp(), message, level});
  if (state_.activity.size() > 20) {
    state_.activity.resize(20);
  }
}

bool EdgeDaemon::RefreshHardwareStateLocked() {
  if (config_.simulate_hardware || !config_.enable_serial) {
    state_.connected = true;
    return true;
  }

  std::string response;
  if (!SendArduinoCommandLocked("STATUS", response, false)) {
    state_.connected = false;
    return false;
  }

  return ParseStatusLineLocked(response);
}

bool EdgeDaemon::ParseStatusLineLocked(const std::string& line) {
  if (!StartsWith(line, "STATUS ")) {
    FailHardwareLocked("Unexpected Arduino status line: " + line);
    return false;
  }

  std::istringstream stream(line.substr(7));
  std::string token;
  std::string mode = state_.mode;
  bool moving = hardware_moving_;
  double yaw = state_.yaw;
  double pitch = state_.pitch;
  int fault = 0;

  try {
    while (stream >> token) {
      const auto separator = token.find('=');
      if (separator == std::string::npos) {
        continue;
      }
      const std::string key = token.substr(0, separator);
      const std::string value = token.substr(separator + 1);
      if (key == "mode") {
        mode = value;
      } else if (key == "moving") {
        moving = value == "1" || value == "true";
      } else if (key == "yaw") {
        yaw = std::stod(value);
      } else if (key == "pitch") {
        pitch = std::stod(value);
      } else if (key == "fault") {
        fault = std::stoi(value);
      }
    }
  } catch (...) {
    FailHardwareLocked("Malformed Arduino status line: " + line);
    return false;
  }

  state_.connected = true;
  state_.mode = mode;
  state_.yaw = Round(yaw, 100.0);
  state_.pitch = Round(pitch, 100.0);
  hardware_moving_ = moving;

  if (fault != 0) {
    state_.faults = {"Arduino fault mask " + std::to_string(fault)};
  } else if (state_.faults.size() == 1 && StartsWith(state_.faults.front(), "Arduino fault mask ")) {
    state_.faults.clear();
  }

  return true;
}

bool EdgeDaemon::SendArduinoCommandLocked(const std::string& line, std::string& response, bool allowSimulation) {
  response.clear();

  if ((config_.simulate_hardware || !config_.enable_serial) && allowSimulation) {
    response = "OK simulated";
    state_.connected = true;
    return true;
  }

  if (!serial_->IsOpen()) {
    response = "Arduino serial port is not connected.";
    state_.connected = false;
    return false;
  }

  if (!serial_->SendCommand(line, response, config_.command_timeout_ms)) {
    response = serial_->last_error();
    state_.connected = false;
    return false;
  }

  state_.connected = true;
  if (StartsWith(response, "ERR ")) {
    response = response.substr(4);
    return false;
  }

  return StartsWith(response, "OK ") || StartsWith(response, "STATUS ");
}

bool EdgeDaemon::IssueMoveToCellLocked(int index, std::string& error_message) {
  error_message.clear();
  const int width = state_.grid.empty() ? 0 : static_cast<int>(state_.grid.front().size());
  const int height = static_cast<int>(state_.grid.size());
  if (width <= 0 || height <= 0) {
    error_message = "Scan grid is not configured.";
    return false;
  }

  const auto [x, y] = CoordForIndex(index, width);
  const double yaw_range = state_.scan_settings.yaw_max - state_.scan_settings.yaw_min;
  const double pitch_range = state_.scan_settings.pitch_max - state_.scan_settings.pitch_min;
  pending_target_yaw_ = state_.scan_settings.yaw_min + (static_cast<double>(x) / std::max(1, width - 1)) * yaw_range;
  pending_target_pitch_ = state_.scan_settings.pitch_min + (static_cast<double>(y) / std::max(1, height - 1)) * pitch_range;

  if (config_.simulate_hardware || !config_.enable_serial) {
    state_.yaw = Round(pending_target_yaw_, 100.0);
    state_.pitch = Round(pending_target_pitch_, 100.0);
    hardware_moving_ = false;
    return true;
  }

  std::string response;
  const std::string command = "MOVE " + FormatCommandFloat(pending_target_yaw_) + " " + FormatCommandFloat(pending_target_pitch_);
  if (!SendArduinoCommandLocked(command, response)) {
    error_message = response;
    return false;
  }

  hardware_moving_ = true;
  return true;
}

bool EdgeDaemon::CaptureCurrentCellLocked(int index, std::string& error_message) {
  error_message.clear();

  std::string response;
  if (!SendArduinoCommandLocked("TRIGGER", response)) {
    error_message = response;
    return false;
  }

  const double distance = lidar_->ReadDistanceMeters(pending_target_yaw_, pending_target_pitch_);
  if (std::isnan(distance)) {
    error_message = "Lidar read failed: " + lidar_->last_error();
    return false;
  }

  const int width = state_.grid.empty() ? 0 : static_cast<int>(state_.grid.front().size());
  const auto [x, y] = CoordForIndex(index, width);
  const double normalized = Clamp((6.4 - distance) / 2.8, 0.0, 1.0);
  state_.grid[y][x] = Round(normalized, 10000.0);
  state_.yaw = Round(pending_target_yaw_, 100.0);
  state_.pitch = Round(pending_target_pitch_, 100.0);
  filled_cells_ = std::max(filled_cells_, index + 1);

  const int total_cells = static_cast<int>(state_.grid.size() * width);
  state_.coverage = Round(static_cast<double>(filled_cells_) / std::max(1, total_cells), 10000.0);
  state_.scan_progress = state_.coverage;
  hardware_moving_ = false;
  return true;
}

void EdgeDaemon::FailHardwareLocked(const std::string& message) {
  state_.connected = false;
  state_.mode = "fault";
  state_.faults = {message};
  hardware_moving_ = false;
  scan_waiting_for_settle_ = false;
  AddLogLocked("hardware", "error", message);
}

void EdgeDaemon::FinishScanLocked(const std::string& message) {
  state_.mode = "idle";
  state_.coverage = 1.0;
  state_.scan_progress = 1.0;
  state_.last_completed_scan_at = CurrentTimestamp();
  hardware_moving_ = false;
  scan_waiting_for_settle_ = false;
  AddLogLocked("scanner", "info", message);
}

std::pair<int, int> EdgeDaemon::CoordForIndex(int index, int width) {
  const int row = width > 0 ? index / width : 0;
  int col = width > 0 ? index % width : 0;
  if (row % 2 == 1) {
    col = (width - 1) - col;
  }
  return {col, row};
}

double EdgeDaemon::Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

double EdgeDaemon::Round(double value, double scale) {
  return std::round(value * scale) / scale;
}

}  // namespace edge
