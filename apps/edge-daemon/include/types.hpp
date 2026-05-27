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
  // Scan motion mode: "sweep" (continuous) or "step" (stop-and-shoot). Mirrors
  // config_.scan.mode; surfaced so the UI can show + toggle it.
  std::string scan_mode = "sweep";
  // Configured sweep velocity ceiling (deg/s); informational for the UI. The
  // actual sweep speed is min(this, cell_width / lidar_period).
  double sweep_max_speed_deg_s = 120.0;
};

// Placeholder for future real sensor telemetry (e.g. measured motor current,
// driver temperature, LIDAR sample rate). The fields that used to live here
// were fabricated numbers the UI displayed as if they were sensor readings;
// they've been removed. Kept as an empty struct so the wire schema has a slot
// to grow into without another breaking change.
struct Metrics {};

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
  std::string mode;  // for set_scan_mode: "sweep" | "step"
};

}  // namespace edge
