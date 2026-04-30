#include "stepper_axis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace edge {
namespace {

double Clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

long DegToMicrosteps(double deg, double microsteps_per_deg) {
  return static_cast<long>(std::llround(deg * microsteps_per_deg));
}

double MicrostepsToDeg(long steps, double microsteps_per_deg) {
  return static_cast<double>(steps) / microsteps_per_deg;
}

// Trapezoidal (or triangular) velocity profile. Returns step times in microseconds
// relative to move start. Assumes a constant microstep (i.e. the planner emits
// one pulse per microstep; the driver board handles microstep interpolation).
std::vector<std::uint32_t> GenerateStepTimes(int N, double accel_steps_s2, double vmax_steps_s) {
  std::vector<std::uint32_t> out;
  if (N <= 0) return out;
  out.reserve(static_cast<std::size_t>(N));

  if (accel_steps_s2 <= 0.0 || vmax_steps_s <= 0.0) {
    return out;  // guarded by validator upstream, but fail-closed here.
  }

  const double vmax_sq = vmax_steps_s * vmax_steps_s;
  int N_accel = static_cast<int>(std::floor(vmax_sq / (2.0 * accel_steps_s2)));
  N_accel = std::min(N_accel, N / 2);
  const int N_cruise = N - 2 * N_accel;

  const double t_accel = (N_accel > 0) ? std::sqrt(2.0 * N_accel / accel_steps_s2) : 0.0;
  const double t_cruise = (N_cruise > 0) ? (static_cast<double>(N_cruise) / vmax_steps_s) : 0.0;
  const double T = 2.0 * t_accel + t_cruise;

  for (int i = 0; i < N; ++i) {
    double t_s;
    if (i < N_accel) {
      t_s = std::sqrt(2.0 * i / accel_steps_s2);
    } else if (i < N_accel + N_cruise) {
      t_s = t_accel + static_cast<double>(i - N_accel) / vmax_steps_s;
    } else {
      const int remaining = N - i;  // 1..N_accel
      t_s = T - std::sqrt(2.0 * remaining / accel_steps_s2);
    }
    if (t_s < 0.0) t_s = 0.0;
    const double t_us = t_s * 1e6;
    out.push_back(static_cast<std::uint32_t>(std::llround(t_us)));
  }

  // Guarantee strict monotonicity even under floating-point jitter.
  for (std::size_t i = 1; i < out.size(); ++i) {
    if (out[i] <= out[i - 1]) out[i] = out[i - 1] + 1;
  }
  return out;
}

}  // namespace

StepperAxis::StepperAxis(AxisId id, double microsteps_per_deg, AxisMotion motion)
    : id_(id), microsteps_per_deg_(microsteps_per_deg), motion_(motion) {}

void StepperAxis::SetMotion(const AxisMotion& motion) { motion_ = motion; }

double StepperAxis::current_deg() const {
  return MicrostepsToDeg(current_microsteps_, microsteps_per_deg_);
}

StepperAxis::MovePlan StepperAxis::PlanMove(double target_deg) const {
  MovePlan plan;
  const double clamped = Clamp(target_deg, motion_.min_deg, motion_.max_deg);
  plan.final_deg = clamped;

  const long target_microsteps = DegToMicrosteps(clamped, microsteps_per_deg_);
  const long delta = target_microsteps - current_microsteps_;
  if (delta == 0) return plan;

  plan.forward = delta > 0;
  plan.steps = static_cast<int>(std::llabs(delta));

  const double accel_steps = motion_.accel_deg_s2 * microsteps_per_deg_;
  const double vmax_steps = motion_.max_speed_deg_s * microsteps_per_deg_;
  plan.step_times_us = GenerateStepTimes(plan.steps, accel_steps, vmax_steps);
  plan.total_duration_us = plan.step_times_us.empty() ? 0u : plan.step_times_us.back();
  return plan;
}

void StepperAxis::Commit(const MovePlan& plan) {
  if (plan.steps == 0) {
    last_target_deg_ = plan.final_deg;
    return;
  }
  current_microsteps_ += (plan.forward ? plan.steps : -plan.steps);
  last_target_deg_ = plan.final_deg;
  position_known_ = true;
}

}  // namespace edge
