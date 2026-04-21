#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"
#include "lidar_sensor.hpp"
#include "motion_controller.hpp"
#include "safety_supervisor.hpp"
#include "types.hpp"

namespace edge {

// EdgeDaemon ties the hardware layer together and exposes a thread-safe
// snapshot + command API to the HTTP server. Long-running scans run in a
// dedicated worker thread; manual jogs and homes block the calling HTTP
// thread (the Go control-api times these calls out at the transport layer).
class EdgeDaemon {
 public:
  explicit EdgeDaemon(Config config);
  ~EdgeDaemon();

  bool Start();
  void Stop();
  bool Healthy() const;

  Snapshot GetSnapshot() const;
  Snapshot ExecuteCommand(const CommandRequest& request, std::string& error_message);

  Config GetConfig() const;
  MotionConfig GetMotionConfig() const;
  // Validates the proposed envelope, applies it to the live MotionController
  // and persists the merged config to disk. Returns "" on success or a human-
  // readable reason suitable for HTTP 400.
  std::string UpdateMotionConfig(const MotionConfig& motion);

 private:
  enum class ScanState { Idle, Scanning, Paused, Stopping };

  void ApplyResolutionLocked(const std::string& resolution);
  void ResetScanLocked();
  void AddLogLocked(const std::string& source, const std::string& level, const std::string& message);
  void UpdateMetricsLocked();
  void FinishScanLocked(const std::string& message);
  void FailLocked(const std::string& message);

  void ScanWorker();
  std::pair<int, int> CoordForIndex(int index, int width) const;
  double TargetYawForCell(int x, int width) const;
  double TargetPitchForCell(int y, int height) const;

  Config config_;
  std::unique_ptr<IGpioBackend> gpio_;
  std::unique_ptr<LidarSensor> lidar_;
  std::unique_ptr<MotionController> motion_;
  std::unique_ptr<SafetySupervisor> safety_;

  mutable std::mutex mutex_;
  Snapshot state_;
  std::atomic<bool> running_{false};

  std::condition_variable scan_cv_;
  std::thread scan_worker_;
  ScanState scan_state_ = ScanState::Idle;
  int scan_index_ = 0;
  int filled_cells_ = 0;
};

}  // namespace edge
