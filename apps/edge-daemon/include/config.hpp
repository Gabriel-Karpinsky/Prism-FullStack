#pragma once

#include <string>

namespace edge {

struct Config {
  std::string bind_host = "127.0.0.1";
  int bind_port = 9090;
  std::string serial_port = "/dev/ttyACM0";
  int serial_baud = 115200;
  bool simulate_hardware = true;
  bool enable_serial = true;
  int grid_width = 48;
  int grid_height = 24;
  int lidar_bus = 1;
  int lidar_address = 0x62;
  int tick_interval_ms = 100;
};

Config LoadConfigFromEnv();

}  // namespace edge
