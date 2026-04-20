#pragma once

#include <cstdint>
#include <vector>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"

namespace edge {

class StepperAxis {
 public:
  struct MovePlan {
    std::vector<std::uint32_t> step_times_us;  // relative to move start
    bool forward = true;
    int steps = 0;
    double final_deg = 0.0;  // clamped target in degrees
    std::uint32_t total_duration_us = 0;
  };

  StepperAxis(AxisId id, double microsteps_per_deg, AxisMotion motion);

  void SetMotion(const AxisMotion& motion);
  const AxisMotion& motion() const { return motion_; }

  double current_deg() const;
  double target_deg() const { return last_target_deg_; }

  // Trapezoidal-profile plan from current position to (clamped) target.
  MovePlan PlanMove(double target_deg) const;

  // Applied once the waveform executed successfully.
  void Commit(const MovePlan& plan);

  // Called after an aborted waveform: position is no longer trusted.
  void MarkPositionUnknown() { position_known_ = false; }
  bool position_known() const { return position_known_; }

  AxisId id() const { return id_; }
  double microsteps_per_deg() const { return microsteps_per_deg_; }

 private:
  AxisId id_;
  double microsteps_per_deg_;
  AxisMotion motion_;
  long current_microsteps_ = 0;
  double last_target_deg_ = 0.0;
  bool position_known_ = true;
};

}  // namespace edge
