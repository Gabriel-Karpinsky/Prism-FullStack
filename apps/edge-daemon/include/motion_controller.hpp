#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"
#include "stepper_axis.hpp"

namespace edge {

class MotionController {
 public:
  struct MoveResult {
    bool success = true;
    std::string error;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
  };

  MotionController(const Config& config, IGpioBackend& gpio);

  // Hot-swap motion envelope. Caller must pre-validate with ValidateMotionConfig
  // AND ensure the current position lies within the new limits.
  void SetMotionConfig(const MotionConfig& motion);
  MotionConfig motion_config() const;

  // Blocking absolute move. Both axes plan independently and run in a single
  // merged DMA waveform; the call returns when the waveform completes or aborts.
  MoveResult MoveTo(double yaw_deg, double pitch_deg);
  MoveResult Home();

  // --- Continuous yaw sweep (for on-the-fly scanning) ---
  // Launches a non-blocking constant-velocity yaw sweep to yaw_target_deg
  // (pitch holds its current position). Returns false on planning/launch error.
  // Usage: StartYawSweep(); while (SweepBusy()) { sample; SweepMicrostepsTravelled(); }
  //        FinishYawSweep(reached_target);
  bool StartYawSweep(double yaw_target_deg, double speed_deg_s, double accel_deg_s2,
                     std::string& error);
  bool SweepBusy() const;
  // Microsteps travelled since the sweep started, by elapsed time against the
  // committed trapezoidal profile (open-loop, exact to the commanded motion).
  long SweepMicrostepsTravelled() const;
  // Commits (reached_target) or invalidates (aborted) the swept position and
  // releases DMA resources.
  void FinishYawSweep(bool reached_target);

  double yaw_deg() const;
  double pitch_deg() const;
  double target_yaw_deg() const;
  double target_pitch_deg() const;
  bool yaw_position_known() const;
  bool pitch_position_known() const;
  bool is_busy() const { return busy_.load(); }

  void AbortMotion();

  void SetEnabled(bool asserted);
  bool enabled() const { return enabled_.load(); }

 private:
  IGpioBackend& gpio_;
  mutable std::mutex mutex_;
  StepperAxis yaw_;
  StepperAxis pitch_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> enabled_{false};

  // Continuous-sweep state (written in StartYawSweep, read by the worker's
  // sampling loop, finalised in FinishYawSweep — all on the scan thread).
  StepperAxis::MovePlan sweep_plan_;
  std::chrono::steady_clock::time_point sweep_start_time_;
  bool sweep_active_ = false;
};

}  // namespace edge
