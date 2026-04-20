#include "hardware_config.hpp"

#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace edge {
namespace {

using json = nlohmann::json;

template <typename T>
T Get(const json& node, const char* key, T fallback) {
  auto it = node.find(key);
  if (it == node.end() || it->is_null()) return fallback;
  try { return it->get<T>(); } catch (...) { return fallback; }
}

AxisMotion ReadAxis(const json& node, const AxisMotion& fallback) {
  if (!node.is_object()) return fallback;
  AxisMotion out = fallback;
  out.min_deg         = Get(node, "min_deg", fallback.min_deg);
  out.max_deg         = Get(node, "max_deg", fallback.max_deg);
  out.max_speed_deg_s = Get(node, "max_speed_deg_s", fallback.max_speed_deg_s);
  out.accel_deg_s2    = Get(node, "accel_deg_s2", fallback.accel_deg_s2);
  return out;
}

json AxisToJson(const AxisMotion& axis) {
  return json{
      {"min_deg", axis.min_deg},
      {"max_deg", axis.max_deg},
      {"max_speed_deg_s", axis.max_speed_deg_s},
      {"accel_deg_s2", axis.accel_deg_s2},
  };
}

json ConfigToJson(const Config& c) {
  return json{
      {"motion", {
          {"yaw", AxisToJson(c.motion.yaw)},
          {"pitch", AxisToJson(c.motion.pitch)},
      }},
      {"mechanics", {
          {"full_steps_per_rev", c.mechanics.full_steps_per_rev},
          {"microsteps", c.mechanics.microsteps},
          {"yaw_gear_ratio", c.mechanics.yaw_gear_ratio},
          {"pitch_gear_ratio", c.mechanics.pitch_gear_ratio},
      }},
      {"gpio", {
          {"yaw_step", c.gpio.yaw_step},
          {"yaw_dir", c.gpio.yaw_dir},
          {"pitch_step", c.gpio.pitch_step},
          {"pitch_dir", c.gpio.pitch_dir},
          {"enable", c.gpio.enable},
          {"lidar_trigger", c.gpio.lidar_trigger},
          {"status_led", c.gpio.status_led},
          {"step_active_low", c.gpio.step_active_low},
          {"dir_active_low", c.gpio.dir_active_low},
          {"enable_active_low", c.gpio.enable_active_low},
      }},
      {"safety", {
          {"host_watchdog_ms", c.safety.host_watchdog_ms},
          {"step_pulse_us", c.safety.step_pulse_us},
          {"lidar_trigger_pulse_us", c.safety.lidar_trigger_pulse_us},
      }},
      {"lidar", {
          {"i2c_bus", c.lidar.i2c_bus},
          {"i2c_address", c.lidar.i2c_address},
          {"simulate", c.lidar.simulate},
      }},
      {"service", {
          {"bind_host", c.service.bind_host},
          {"bind_port", c.service.bind_port},
          {"grid_width", c.service.grid_width},
          {"grid_height", c.service.grid_height},
          {"tick_interval_ms", c.service.tick_interval_ms},
          {"status_broadcast_interval_ms", c.service.status_broadcast_interval_ms},
      }},
      {"simulate_hardware", c.simulate_hardware},
  };
}

}  // namespace

double MechanicsConfig::yaw_microsteps_per_deg() const {
  return (static_cast<double>(full_steps_per_rev) * microsteps * yaw_gear_ratio) / 360.0;
}

double MechanicsConfig::pitch_microsteps_per_deg() const {
  return (static_cast<double>(full_steps_per_rev) * microsteps * pitch_gear_ratio) / 360.0;
}

Config DefaultConfig() {
  Config c{};
  c.motion.yaw   = {-50.0,  50.0, 18.0, 60.0};
  c.motion.pitch = {-30.0,  30.0, 12.0, 40.0};
  c.mechanics = {200, 128, 1.0, 1.0};
  c.gpio = {17, 27, 22, 23, 24, 25, 18,
            /*step_active_low=*/true,
            /*dir_active_low=*/false,
            /*enable_active_low=*/true};
  c.safety = {1500, 4, 25};
  c.lidar = {1, 0x62, false};
  c.service = {"127.0.0.1", 9090, 48, 24, 20, 100};
  c.simulate_hardware = false;
  return c;
}

Config LoadConfigFromFile(const std::string& path) {
  Config c = DefaultConfig();
  c.config_file_path = path;

  std::ifstream in(path);
  if (!in.good()) return c;

  json doc;
  try { in >> doc; } catch (...) { return c; }
  if (!doc.is_object()) return c;

  if (auto it = doc.find("motion"); it != doc.end() && it->is_object()) {
    c.motion.yaw   = ReadAxis((*it)["yaw"],   c.motion.yaw);
    c.motion.pitch = ReadAxis((*it)["pitch"], c.motion.pitch);
  }
  if (auto it = doc.find("mechanics"); it != doc.end() && it->is_object()) {
    c.mechanics.full_steps_per_rev = Get(*it, "full_steps_per_rev", c.mechanics.full_steps_per_rev);
    c.mechanics.microsteps         = Get(*it, "microsteps",         c.mechanics.microsteps);
    c.mechanics.yaw_gear_ratio     = Get(*it, "yaw_gear_ratio",     c.mechanics.yaw_gear_ratio);
    c.mechanics.pitch_gear_ratio   = Get(*it, "pitch_gear_ratio",   c.mechanics.pitch_gear_ratio);
  }
  if (auto it = doc.find("gpio"); it != doc.end() && it->is_object()) {
    c.gpio.yaw_step          = Get<unsigned>(*it, "yaw_step",          c.gpio.yaw_step);
    c.gpio.yaw_dir           = Get<unsigned>(*it, "yaw_dir",           c.gpio.yaw_dir);
    c.gpio.pitch_step        = Get<unsigned>(*it, "pitch_step",        c.gpio.pitch_step);
    c.gpio.pitch_dir         = Get<unsigned>(*it, "pitch_dir",         c.gpio.pitch_dir);
    c.gpio.enable            = Get<unsigned>(*it, "enable",            c.gpio.enable);
    c.gpio.lidar_trigger     = Get<unsigned>(*it, "lidar_trigger",     c.gpio.lidar_trigger);
    c.gpio.status_led        = Get<unsigned>(*it, "status_led",        c.gpio.status_led);
    c.gpio.step_active_low   = Get(*it, "step_active_low",   c.gpio.step_active_low);
    c.gpio.dir_active_low    = Get(*it, "dir_active_low",    c.gpio.dir_active_low);
    c.gpio.enable_active_low = Get(*it, "enable_active_low", c.gpio.enable_active_low);
  }
  if (auto it = doc.find("safety"); it != doc.end() && it->is_object()) {
    c.safety.host_watchdog_ms       = Get(*it, "host_watchdog_ms",       c.safety.host_watchdog_ms);
    c.safety.step_pulse_us          = Get(*it, "step_pulse_us",          c.safety.step_pulse_us);
    c.safety.lidar_trigger_pulse_us = Get(*it, "lidar_trigger_pulse_us", c.safety.lidar_trigger_pulse_us);
  }
  if (auto it = doc.find("lidar"); it != doc.end() && it->is_object()) {
    c.lidar.i2c_bus     = Get(*it, "i2c_bus",     c.lidar.i2c_bus);
    c.lidar.i2c_address = Get(*it, "i2c_address", c.lidar.i2c_address);
    c.lidar.simulate    = Get(*it, "simulate",    c.lidar.simulate);
  }
  if (auto it = doc.find("service"); it != doc.end() && it->is_object()) {
    c.service.bind_host                   = Get<std::string>(*it, "bind_host", c.service.bind_host);
    c.service.bind_port                   = Get(*it, "bind_port",                   c.service.bind_port);
    c.service.grid_width                  = Get(*it, "grid_width",                  c.service.grid_width);
    c.service.grid_height                 = Get(*it, "grid_height",                 c.service.grid_height);
    c.service.tick_interval_ms            = Get(*it, "tick_interval_ms",            c.service.tick_interval_ms);
    c.service.status_broadcast_interval_ms = Get(*it, "status_broadcast_interval_ms", c.service.status_broadcast_interval_ms);
  }
  c.simulate_hardware = Get(doc, "simulate_hardware", c.simulate_hardware);
  return c;
}

void SaveConfigToFile(const Config& config, const std::string& path) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out.good()) {
      throw std::runtime_error("cannot open " + tmp + " for writing");
    }
    out << ConfigToJson(config).dump(2) << '\n';
    if (!out.good()) {
      throw std::runtime_error("write to " + tmp + " failed");
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    throw std::runtime_error("rename " + tmp + " -> " + path + " failed: " + ec.message());
  }
}

std::string ValidateMotionConfig(const MotionConfig& m) {
  auto validateAxis = [](const AxisMotion& a, const char* name) -> std::string {
    if (!(a.max_deg > a.min_deg)) {
      return std::string(name) + ".max_deg must be greater than min_deg";
    }
    if (!(a.max_speed_deg_s > 0.0)) {
      return std::string(name) + ".max_speed_deg_s must be positive";
    }
    if (!(a.accel_deg_s2 > 0.0)) {
      return std::string(name) + ".accel_deg_s2 must be positive";
    }
    return {};
  };
  if (auto e = validateAxis(m.yaw,   "yaw");   !e.empty()) return e;
  if (auto e = validateAxis(m.pitch, "pitch"); !e.empty()) return e;
  return {};
}

std::string ResolveConfigPath() {
  if (const char* env = std::getenv("PRISM_HARDWARE_CONFIG"); env && *env) {
    return env;
  }
  return "/etc/prism-scanner/hardware.json";
}

}  // namespace edge
