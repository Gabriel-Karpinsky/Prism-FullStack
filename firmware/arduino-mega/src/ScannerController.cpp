#include "ScannerController.h"

#include <math.h>

namespace scanner {
namespace {

uint32_t SafeInterval(uint32_t intervalMicros) {
  return intervalMicros < 150 ? 150 : intervalMicros;
}

}  // namespace

void ScannerController::Begin() {
  pinMode(yaw_axis_.step_pin, OUTPUT);
  pinMode(yaw_axis_.dir_pin, OUTPUT);
  pinMode(pitch_axis_.step_pin, OUTPUT);
  pinMode(pitch_axis_.dir_pin, OUTPUT);
  pinMode(hw::kDriverEnablePin, OUTPUT);
  pinMode(hw::kTriggerPin, OUTPUT);
  pinMode(hw::kStatusLedPin, OUTPUT);

  digitalWrite(yaw_axis_.step_pin, LOW);
  digitalWrite(yaw_axis_.dir_pin, LOW);
  digitalWrite(pitch_axis_.step_pin, LOW);
  digitalWrite(pitch_axis_.dir_pin, LOW);
  digitalWrite(hw::kTriggerPin, LOW);
  digitalWrite(hw::kStatusLedPin, LOW);

  yaw_axis_.step_interval_us = SafeInterval(yaw_axis_.step_interval_us);
  pitch_axis_.step_interval_us = SafeInterval(pitch_axis_.step_interval_us);
  yaw_axis_.current_steps = yaw_axis_.target_steps = 0;
  pitch_axis_.current_steps = pitch_axis_.target_steps = 0;
  yaw_axis_.pulse_high = false;
  pitch_axis_.pulse_high = false;
  trigger_count_ = 0;
  trigger_pulse_until_us_ = 0;
  fault_mask_ = 0;
  mode_ = MotionMode::kIdle;
  last_host_seen_ms_ = millis();

  SetDriverEnabled(true);
}

void ScannerController::Update() {
  const uint32_t nowMicros = micros();
  const uint32_t nowMillis = millis();

  if (trigger_pulse_until_us_ != 0 && TimeReached(nowMicros, trigger_pulse_until_us_)) {
    digitalWrite(hw::kTriggerPin, LOW);
    trigger_pulse_until_us_ = 0;
  }

  if (mode_ != MotionMode::kFault &&
      (mode_ == MotionMode::kManual || mode_ == MotionMode::kScanning || mode_ == MotionMode::kPaused) &&
      static_cast<uint32_t>(nowMillis - last_host_seen_ms_) > hw::kHostWatchdogMs) {
    fault_mask_ |= kFaultWatchdog;
    mode_ = MotionMode::kFault;
    StopMotionInternal();
    SetDriverEnabled(false);
  }

  if (mode_ != MotionMode::kFault) {
    UpdateAxis(yaw_axis_, nowMicros);
    UpdateAxis(pitch_axis_, nowMicros);
  }

  if (mode_ == MotionMode::kFault) {
    digitalWrite(hw::kStatusLedPin, (nowMillis / 250U) % 2U ? HIGH : LOW);
  } else {
    digitalWrite(hw::kStatusLedPin, moving() ? HIGH : LOW);
  }
}

bool ScannerController::Home() {
  if (mode_ == MotionMode::kFault) {
    return false;
  }
  return MoveInternal(0.0f, 0.0f, MotionMode::kManual);
}

bool ScannerController::MoveTo(float yawDeg, float pitchDeg) {
  if (mode_ == MotionMode::kFault || mode_ == MotionMode::kPaused) {
    return false;
  }
  const MotionMode requestedMode = mode_ == MotionMode::kScanning ? MotionMode::kScanning : MotionMode::kManual;
  return MoveInternal(yawDeg, pitchDeg, requestedMode);
}

bool ScannerController::JogYaw(float deltaDeg) {
  return MoveTo(target_yaw_deg() + deltaDeg, target_pitch_deg());
}

bool ScannerController::JogPitch(float deltaDeg) {
  return MoveTo(target_yaw_deg(), target_pitch_deg() + deltaDeg);
}

bool ScannerController::StartScan() {
  if (mode_ == MotionMode::kFault) {
    return false;
  }
  mode_ = MotionMode::kScanning;
  last_host_seen_ms_ = millis();
  return true;
}

bool ScannerController::PauseScan() {
  if (mode_ != MotionMode::kScanning) {
    return false;
  }
  StopMotionInternal();
  mode_ = MotionMode::kPaused;
  last_host_seen_ms_ = millis();
  return true;
}

bool ScannerController::ResumeScan() {
  if (mode_ != MotionMode::kPaused) {
    return false;
  }
  mode_ = MotionMode::kScanning;
  last_host_seen_ms_ = millis();
  return true;
}

bool ScannerController::StopScan() {
  if (mode_ == MotionMode::kFault) {
    return false;
  }
  StopMotionInternal();
  mode_ = MotionMode::kIdle;
  last_host_seen_ms_ = millis();
  return true;
}

bool ScannerController::Trigger() {
  if (mode_ == MotionMode::kFault) {
    return false;
  }
  digitalWrite(hw::kTriggerPin, HIGH);
  trigger_pulse_until_us_ = micros() + hw::kGarminTriggerPulseMicros;
  ++trigger_count_;
  last_host_seen_ms_ = millis();
  return true;
}

void ScannerController::Heartbeat() { last_host_seen_ms_ = millis(); }

void ScannerController::EStop() {
  fault_mask_ |= kFaultEStop;
  mode_ = MotionMode::kFault;
  StopMotionInternal();
  SetDriverEnabled(false);
}

bool ScannerController::ClearFault() {
  fault_mask_ = 0;
  mode_ = MotionMode::kIdle;
  trigger_pulse_until_us_ = 0;
  digitalWrite(hw::kTriggerPin, LOW);
  SetDriverEnabled(true);
  last_host_seen_ms_ = millis();
  return true;
}

ControllerStatus ScannerController::status() const {
  ControllerStatus status;
  status.mode = mode_;
  status.moving = moving();
  status.yaw_deg = yaw_deg();
  status.pitch_deg = pitch_deg();
  status.target_yaw_deg = target_yaw_deg();
  status.target_pitch_deg = target_pitch_deg();
  status.fault_mask = fault_mask_;
  status.trigger_count = trigger_count_;
  return status;
}

bool ScannerController::moving() const {
  return yaw_axis_.current_steps != yaw_axis_.target_steps || pitch_axis_.current_steps != pitch_axis_.target_steps ||
         yaw_axis_.pulse_high || pitch_axis_.pulse_high;
}

float ScannerController::yaw_deg() const {
  return StepsToDegrees(yaw_axis_.current_steps, yaw_axis_.microsteps_per_degree);
}

float ScannerController::pitch_deg() const {
  return StepsToDegrees(pitch_axis_.current_steps, pitch_axis_.microsteps_per_degree);
}

float ScannerController::target_yaw_deg() const {
  return StepsToDegrees(yaw_axis_.target_steps, yaw_axis_.microsteps_per_degree);
}

float ScannerController::target_pitch_deg() const {
  return StepsToDegrees(pitch_axis_.target_steps, pitch_axis_.microsteps_per_degree);
}

bool ScannerController::MoveInternal(float yawDeg, float pitchDeg, MotionMode requestedMode) {
  yawDeg = ClampFloat(yawDeg, yaw_axis_.min_deg, yaw_axis_.max_deg);
  pitchDeg = ClampFloat(pitchDeg, pitch_axis_.min_deg, pitch_axis_.max_deg);

  yaw_axis_.target_steps = DegreesToSteps(yawDeg, yaw_axis_.microsteps_per_degree);
  pitch_axis_.target_steps = DegreesToSteps(pitchDeg, pitch_axis_.microsteps_per_degree);
  mode_ = requestedMode;
  last_host_seen_ms_ = millis();
  return true;
}

void ScannerController::StopMotionInternal() {
  yaw_axis_.target_steps = yaw_axis_.current_steps;
  pitch_axis_.target_steps = pitch_axis_.current_steps;
  yaw_axis_.pulse_high = false;
  pitch_axis_.pulse_high = false;
  digitalWrite(yaw_axis_.step_pin, LOW);
  digitalWrite(pitch_axis_.step_pin, LOW);
}

void ScannerController::SetDriverEnabled(bool enabled) {
  const uint8_t level = (enabled == hw::kEnableActiveLow) ? LOW : HIGH;
  digitalWrite(hw::kDriverEnablePin, level);
}

void ScannerController::UpdateAxis(AxisState& axis, uint32_t nowMicros) {
  if (axis.pulse_high && TimeReached(nowMicros, axis.pulse_low_deadline_us)) {
    digitalWrite(axis.step_pin, LOW);
    axis.pulse_high = false;
  }

  if (axis.current_steps == axis.target_steps || axis.pulse_high || !TimeReached(nowMicros, axis.next_step_due_us)) {
    return;
  }

  const bool forward = axis.target_steps > axis.current_steps;
  digitalWrite(axis.dir_pin, forward ? HIGH : LOW);
  digitalWrite(axis.step_pin, HIGH);
  axis.pulse_high = true;
  axis.pulse_low_deadline_us = nowMicros + hw::kStepPulseWidthMicros;
  axis.next_step_due_us = nowMicros + axis.step_interval_us;
  axis.current_steps += forward ? 1 : -1;
}

bool ScannerController::TimeReached(uint32_t nowMicros, uint32_t deadlineMicros) {
  return static_cast<int32_t>(nowMicros - deadlineMicros) >= 0;
}

float ScannerController::ClampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

long ScannerController::DegreesToSteps(float degrees, float microstepsPerDegree) {
  return lroundf(degrees * microstepsPerDegree);
}

float ScannerController::StepsToDegrees(long steps, float microstepsPerDegree) {
  return static_cast<float>(steps) / microstepsPerDegree;
}

}  // namespace scanner
