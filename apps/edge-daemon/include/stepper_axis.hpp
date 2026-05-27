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
  // speed_deg_s / accel_deg_s2 of 0 mean "use this axis's motion envelope";
  // pass overrides to plan a faster continuous sweep.
  MovePlan PlanMove(double target_deg, double speed_deg_s = 0.0, double accel_deg_s2 = 0.0) const;

  // Applied once the waveform executed successfully.
  void Commit(const MovePlan& plan);

  // Called after an aborted waveform: position is no longer trusted.
  void MarkPositionUnknown() { position_known_ = false; }
  bool position_known() const { return position_known_; }

  // Re-establish a known datum without motion. With no endstops, the only way
  // to recover after position tracking is lost is for the operator to hand-zero
  // the gantry and then declare the current physical pose as zero. Restores
  // tracking so moves are permitted again (B6 recovery path).
  void ResetToZero() {
    current_microsteps_ = 0;
    last_target_deg_ = 0.0;
    position_known_ = true;
  }

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
