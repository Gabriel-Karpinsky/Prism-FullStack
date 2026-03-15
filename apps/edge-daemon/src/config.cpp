#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace edge {
namespace {

std::string ReadEnv(const char* key, const std::string& fallback) {
  if (const char* value = std::getenv(key)) {
    return value;
  }
  return fallback;
}

int ReadEnvInt(const char* key, int fallback) {
  if (const char* value = std::getenv(key)) {
    try {
      return std::stoi(value);
    } catch (...) {
      return fallback;
    }
  }
  return fallback;
}

bool ReadEnvBool(const char* key, bool fallback) {
  std::string value = ReadEnv(key, fallback ? "1" : "0");
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

}  // namespace

Config LoadConfigFromEnv() {
  Config config;
  config.bind_host = ReadEnv("EDGE_BIND_HOST", config.bind_host);
  config.bind_port = ReadEnvInt("EDGE_BIND_PORT", config.bind_port);
  config.serial_port = ReadEnv("EDGE_SERIAL_PORT", config.serial_port);
  config.serial_baud = ReadEnvInt("EDGE_SERIAL_BAUD", config.serial_baud);
  config.simulate_hardware = ReadEnvBool("EDGE_USE_SIMULATION", config.simulate_hardware);
  config.enable_serial = ReadEnvBool("EDGE_ENABLE_SERIAL", config.enable_serial);
  config.grid_width = ReadEnvInt("EDGE_GRID_WIDTH", config.grid_width);
  config.grid_height = ReadEnvInt("EDGE_GRID_HEIGHT", config.grid_height);
  config.lidar_bus = ReadEnvInt("EDGE_LIDAR_BUS", config.lidar_bus);
  config.lidar_address = ReadEnvInt("EDGE_LIDAR_ADDRESS", config.lidar_address);
  config.tick_interval_ms = ReadEnvInt("EDGE_TICK_INTERVAL_MS", config.tick_interval_ms);
  return config;
}

}  // namespace edge
