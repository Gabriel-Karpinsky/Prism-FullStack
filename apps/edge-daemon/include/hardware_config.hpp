#pragma once

#include <cstdint>
#include <string>

namespace edge {

struct AxisMotion {
  double min_deg;
  double max_deg;
  double max_speed_deg_s;
  double accel_deg_s2;
};

struct MotionConfig {
  AxisMotion yaw;
  AxisMotion pitch;
};

struct MechanicsConfig {
  int full_steps_per_rev;
  int microsteps;
  double yaw_gear_ratio;
  double pitch_gear_ratio;

  double yaw_microsteps_per_deg() const;
  double pitch_microsteps_per_deg() const;
};

struct GpioConfig {
  unsigned yaw_step;
  unsigned yaw_dir;
  unsigned pitch_step;
  unsigned pitch_dir;
  unsigned enable;
  unsigned lidar_trigger;
  unsigned status_led;
  bool step_active_low;
  bool dir_active_low;
  bool enable_active_low;
};

struct SafetyConfig {
  int host_watchdog_ms;
  int step_pulse_us;
  int lidar_trigger_pulse_us;
};

struct LidarHardwareConfig {
  int i2c_bus;
  int i2c_address;
  bool simulate;
};

struct ServiceConfig {
  std::string bind_host;
  int bind_port;
  int grid_width;
  int grid_height;
  int tick_interval_ms;
  int status_broadcast_interval_ms;
};

struct Config {
  MotionConfig motion;
  MechanicsConfig mechanics;
  GpioConfig gpio;
  SafetyConfig safety;
  LidarHardwareConfig lidar;
  ServiceConfig service;
  bool simulate_hardware;
  std::string config_file_path;
};

Config DefaultConfig();

// Loads config from the given JSON file. Missing or malformed fields fall back
// to DefaultConfig(); a file that cannot be opened returns DefaultConfig() with
// config_file_path set so later writes can still persist.
Config LoadConfigFromFile(const std::string& path);

// Atomic write: serialises to path + ".tmp" then renames. Throws std::runtime_error on failure.
void SaveConfigToFile(const Config& config, const std::string& path);

// Returns empty string if valid; otherwise a human-readable reason suitable for HTTP 400.
std::string ValidateMotionConfig(const MotionConfig& motion);

// Resolves config file path from env (PRISM_HARDWARE_CONFIG) or returns default /etc/prism-scanner/hardware.json.
std::string ResolveConfigPath();

}  // namespace edge
