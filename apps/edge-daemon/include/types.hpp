#pragma once

#include <cstdint>
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
  // Resolution preset: coarse | standard | fine | max. Each maps to a
  // sampling stride in microsteps (see EdgeDaemon::ApplyResolutionLocked).
  std::string resolution = "standard";
  // Microsteps the head advances between samples; derived from the preset.
  int sample_stride_microsteps = 0;
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
  // The scan grid is no longer carried in the Snapshot — it can be hundreds of
  // thousands of cells. It's fetched incrementally via GridUpdate (see below)
  // so a poll only ships the cells that changed since the client's last version.
};

// Incremental scan-grid update. The client holds the grid locally and applies
// deltas: each poll sends ?since=<version>&gen=<generation>, and we return only
// the cells whose value changed since that version (or all filled cells when
// the client's generation is stale — e.g. after a resolution change or scan
// reset, which bump the generation). idx/val are parallel arrays; idx is the
// row-major cell index (y*width + x), val the normalised 0..1 height.
struct GridUpdate {
  std::uint64_t generation = 0;  // grid identity; changes on resize/reset
  std::uint64_t version = 0;     // monotonic change counter (resets on generation bump)
  int width = 0;
  int height = 0;
  bool full = false;             // true ⇒ client should rebuild from idx/val
  std::vector<int> idx;
  std::vector<double> val;
};

struct CommandRequest {
  std::string command;
  std::string axis;
  double delta = 0.0;
  std::string resolution;
};

}  // namespace edge
