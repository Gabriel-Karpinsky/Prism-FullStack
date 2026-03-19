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
  bool simulate_lidar = true;
  int grid_width = 48;
  int grid_height = 24;
  int lidar_bus = 1;
  int lidar_address = 0x62;
  int tick_interval_ms = 20;
  int status_poll_interval_ms = 100;
  int heartbeat_interval_ms = 400;
  int command_timeout_ms = 900;
  int move_settle_ms = 80;
  int estimated_point_time_ms = 120;
};

Config LoadConfigFromEnv();

}  // namespace edge
