#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"
#include "lidar_sensor.hpp"
#include "motion_controller.hpp"
#include "safety_supervisor.hpp"
#include "types.hpp"

namespace edge {

// EdgeDaemon ties the hardware layer together and exposes a thread-safe
// snapshot + command API to the HTTP server. Long-running scans run in a
// dedicated worker thread; manual jogs and homes are likewise offloaded to a
// move worker (B8) so they return immediately with mode "manual" and the UI
// polls /api/state for completion — a multi-second home no longer blocks all
// polling (which would also starve the safety heartbeat).
class EdgeDaemon {
 public:
  explicit EdgeDaemon(Config config);
  ~EdgeDaemon();

  bool Start();
  void Stop();
  bool Healthy() const;

  Snapshot GetSnapshot() const;
  Snapshot ExecuteCommand(const CommandRequest& request, std::string& error_message);

  // Incremental scan-grid accessor. Returns the cells that changed since
  // `since_version`, or all filled cells when `client_generation` is stale.
  // Lets the HTTP layer ship deltas instead of the whole grid every poll.
  GridUpdate GetGridUpdate(std::uint64_t since_version, std::uint64_t client_generation) const;

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

  void ScanWorker();        // dispatches to step or sweep loop by config_.scan.mode
  void ScanWorkerSweep();   // continuous: sweep each row while sampling on the fly
  void ManualMoveWorker(double yaw_deg, double pitch_deg, bool is_home);  // async home/jog (B8)

  // Grid helpers (all require mutex_ held). RebuildGridLocked resizes to w×h,
  // clears to empty (-1) and bumps the generation; ClearGridLocked clears in
  // place (also a new generation); MarkCellLocked writes one cell + stamps its
  // version so GetGridUpdate can ship it as a delta.
  void RebuildGridLocked(int width, int height);
  void ClearGridLocked();
  void MarkCellLocked(int x, int y, double value);
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
  std::thread move_worker_;            // runs an async home/jog (B8)
  bool manual_move_active_ = false;    // guarded by mutex_; a move worker is in flight
  ScanState scan_state_ = ScanState::Idle;
  int scan_index_ = 0;   // cell cursor (step mode)
  int scan_row_ = 0;     // row cursor (sweep mode)
  int filled_cells_ = 0;

  // Scan grid + incremental-update bookkeeping (guarded by mutex_). The grid is
  // no longer part of Snapshot; it lives here and is shipped as deltas. cell_version_
  // is flat (row-major, size grid_w_*grid_h_) and stores the grid_version_ at which
  // each cell last changed.
  std::vector<std::vector<double>> grid_;
  std::vector<std::uint64_t> cell_version_;
  std::uint64_t grid_generation_ = 0;
  std::uint64_t grid_version_ = 0;
  int grid_w_ = 0;
  int grid_h_ = 0;
};

}  // namespace edge
