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
  void FillToLocked(int target_filled);
  void SetHeadLocked(int index);
  void UpdateMetricsLocked();
  void AddLogLocked(const std::string& source, const std::string& level, const std::string& message);
  bool SendPrototypeCommandLocked(const CommandRequest& request, std::string& error_message);
  static double ScanDurationForResolution(const std::string& resolution);
  static double Clamp(double value, double min_value, double max_value);
  static double Round(double value, double scale);

  Config config_;
  mutable std::mutex mutex_;
  Snapshot state_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::unique_ptr<SerialTransport> serial_;
  std::unique_ptr<LidarSensor> lidar_;
  std::chrono::steady_clock::time_point scan_started_at_{};
  double scan_accumulated_seconds_ = 0.0;
  int filled_cells_ = 0;
};

}  // namespace edge
