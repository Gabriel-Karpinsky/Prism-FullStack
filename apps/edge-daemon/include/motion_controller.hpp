#pragma once

#include <atomic>
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
};

}  // namespace edge
