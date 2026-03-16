#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "config.hpp"
#include "lidar_sensor.hpp"
#include "serial_transport.hpp"
#include "types.hpp"

namespace edge {

class EdgeDaemon {
 public:
  explicit EdgeDaemon(Config config);
  ~EdgeDaemon();

  bool Start();
  void Stop();

  Snapshot GetSnapshot() const;
  Snapshot ExecuteCommand(const CommandRequest& request, std::string& error_message);
  bool Healthy() const;

 private:
  void RunLoop();
  void Tick();
  void ResetScanLocked();
  void ApplyResolutionLocked(const std::string& resolution);
  void UpdateMetricsLocked();
  void AddLogLocked(const std::string& source, const std::string& level, const std::string& message);
  bool RefreshHardwareStateLocked();
  bool ParseStatusLineLocked(const std::string& line);
  bool SendArduinoCommandLocked(const std::string& line, std::string& response, bool allowSimulation = true);
  bool IssueMoveToCellLocked(int index, std::string& error_message);
  bool CaptureCurrentCellLocked(int index, std::string& error_message);
  void FailHardwareLocked(const std::string& message);
  void FinishScanLocked(const std::string& message);
  static std::pair<int, int> CoordForIndex(int index, int width);
  static double Clamp(double value, double min_value, double max_value);
  static double Round(double value, double scale);

  Config config_;
  mutable std::mutex mutex_;
  Snapshot state_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::unique_ptr<SerialTransport> serial_;
  std::unique_ptr<LidarSensor> lidar_;
  std::chrono::steady_clock::time_point last_status_poll_at_{};
  std::chrono::steady_clock::time_point last_heartbeat_at_{};
  std::chrono::steady_clock::time_point last_move_issued_at_{};
  int filled_cells_ = 0;
  int current_scan_index_ = 0;
  bool scan_waiting_for_settle_ = false;
  bool hardware_moving_ = false;
  double pending_target_yaw_ = 0.0;
  double pending_target_pitch_ = 0.0;
};

}  // namespace edge
