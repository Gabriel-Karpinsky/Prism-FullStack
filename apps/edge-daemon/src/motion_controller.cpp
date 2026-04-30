#include "motion_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <vector>

namespace edge {
namespace {

WaveformPlan MergeAxisPlans(const StepperAxis::MovePlan& yaw_plan,
                            const StepperAxis::MovePlan& pitch_plan) {
  WaveformPlan wave;
  wave.yaw_forward = yaw_plan.forward;
  wave.pitch_forward = pitch_plan.forward;
  wave.yaw_steps_signed   = yaw_plan.forward   ?  yaw_plan.steps   : -yaw_plan.steps;
  wave.pitch_steps_signed = pitch_plan.forward ?  pitch_plan.steps : -pitch_plan.steps;

  wave.pulses.reserve(yaw_plan.step_times_us.size() + pitch_plan.step_times_us.size());
  for (auto t : yaw_plan.step_times_us)   wave.pulses.push_back({t, 0b01});
  for (auto t : pitch_plan.step_times_us) wave.pulses.push_back({t, 0b10});

  std::sort(wave.pulses.begin(), wave.pulses.end(),
            [](const StepPulse& a, const StepPulse& b) { return a.time_us < b.time_us; });

  // Coincident pulses (same microsecond) merge into a combined mask so pigpio
  // sees them as a single gpioPulse_t. Distinct times stay separate.
  std::vector<StepPulse> merged;
  merged.reserve(wave.pulses.size());
  for (const auto& p : wave.pulses) {
    if (!merged.empty() && merged.back().time_us == p.time_us) {
      merged.back().axis_mask |= p.axis_mask;
    } else {
      merged.push_back(p);
    }
  }
  wave.pulses = std::move(merged);
  return wave;
}

}  // namespace

MotionController::MotionController(const Config& config, IGpioBackend& gpio)
    : gpio_(gpio),
      yaw_(AxisId::Yaw,   config.mechanics.yaw_microsteps_per_deg(),   config.motion.yaw),
      pitch_(AxisId::Pitch, config.mechanics.pitch_microsteps_per_deg(), config.motion.pitch) {}

void MotionController::SetMotionConfig(const MotionConfig& motion) {
  std::lock_guard<std::mutex> lock(mutex_);
  yaw_.SetMotion(motion.yaw);
  pitch_.SetMotion(motion.pitch);
}

MotionConfig MotionController::motion_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  MotionConfig m;
  m.yaw = yaw_.motion();
  m.pitch = pitch_.motion();
  return m;
}

MotionController::MoveResult MotionController::MoveTo(double yaw_deg, double pitch_deg) {
  MoveResult result;
  std::unique_lock<std::mutex> lock(mutex_);

  if (busy_.load()) {
    result.success = false;
    result.error = "motion already in flight";
    result.yaw_deg = yaw_.current_deg();
    result.pitch_deg = pitch_.current_deg();
    return result;
  }

  const auto yaw_plan = yaw_.PlanMove(yaw_deg);
  const auto pitch_plan = pitch_.PlanMove(pitch_deg);

  if (yaw_plan.steps == 0 && pitch_plan.steps == 0) {
    result.yaw_deg = yaw_.current_deg();
    result.pitch_deg = pitch_.current_deg();
    return result;
  }

  gpio_.SetAxisDirection(AxisId::Yaw,   yaw_plan.forward);
  gpio_.SetAxisDirection(AxisId::Pitch, pitch_plan.forward);
  if (!enabled_.load()) {
    gpio_.SetEnabled(true);
    enabled_.store(true);
  }

  WaveformPlan wave = MergeAxisPlans(yaw_plan, pitch_plan);

  busy_.store(true);
  lock.unlock();  // free the lock while the DMA waveform runs; AbortMotion does not need it.

  std::string err;
  const bool ok = gpio_.RunMotionWaveform(wave, err);

  lock.lock();
  busy_.store(false);

  if (ok) {
    yaw_.Commit(yaw_plan);
    pitch_.Commit(pitch_plan);
  } else {
    yaw_.MarkPositionUnknown();
    pitch_.MarkPositionUnknown();
    result.success = false;
    result.error = err.empty() ? "motion aborted" : err;
  }
  result.yaw_deg = yaw_.current_deg();
  result.pitch_deg = pitch_.current_deg();
  return result;
}

MotionController::MoveResult MotionController::Home() {
  // No endstops: "home" is a soft zero — move to (0,0) and trust the axis state.
  // The operator is expected to have hand-zeroed the gantry before service start.
  return MoveTo(0.0, 0.0);
}

double MotionController::yaw_deg() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_.current_deg();
}
double MotionController::pitch_deg() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pitch_.current_deg();
}
double MotionController::target_yaw_deg() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_.target_deg();
}
double MotionController::target_pitch_deg() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pitch_.target_deg();
}
bool MotionController::yaw_position_known() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_.position_known();
}
bool MotionController::pitch_position_known() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pitch_.position_known();
}

void MotionController::AbortMotion() { gpio_.AbortMotion(); }

void MotionController::SetEnabled(bool asserted) {
  gpio_.SetEnabled(asserted);
  enabled_.store(asserted);
}

}  // namespace edge
