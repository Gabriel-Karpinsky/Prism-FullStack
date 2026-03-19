#pragma once

#include <optional>
#include <string>
#include <vector>

namespace edge {

struct ActivityEntry {
  std::string source;
  std::string ts;
  std::string message;
  std::string level;
};

struct ScanSettings {
  double yaw_min = -60.0;
  double yaw_max = 60.0;
  double pitch_min = -20.0;
  double pitch_max = 35.0;
  double sweep_speed_deg_per_sec = 20.0;
  std::string resolution = "medium";
};

struct Metrics {
  double motor_temp_c = 31.0;
  double motor_current_a = 1.2;
  int lidar_fps = 18;
  int radar_fps = 0;
  int latency_ms = 40;
  int packets_dropped = 0;
};

struct Snapshot {
  bool connected = true;
  std::string mode = "idle";
  std::string control_owner;
  std::optional<std::string> control_lease_expires_at;
  double yaw = 0.0;
  double pitch = 0.0;
  double coverage = 0.0;
  double scan_progress = 0.0;
  double scan_duration_seconds = 36.0;
  std::optional<std::string> last_completed_scan_at;
  ScanSettings scan_settings;
  Metrics metrics;
  std::vector<std::string> faults;
  std::vector<ActivityEntry> activity;
  std::vector<std::vector<double>> grid;
};

struct CommandRequest {
  std::string command;
  std::string axis;
  double delta = 0.0;
  std::string resolution;
};

}  // namespace edge
