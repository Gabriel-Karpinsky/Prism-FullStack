#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

namespace scanner {

enum class MotionMode : uint8_t {
  kIdle = 0,
  kManual = 1,
  kScanning = 2,
  kPaused = 3,
  kFault = 4,
};

struct ControllerStatus {
  MotionMode mode = MotionMode::kIdle;
  bool moving = false;
  float yaw_deg = 0.0f;
  float pitch_deg = 0.0f;
  float target_yaw_deg = 0.0f;
  float target_pitch_deg = 0.0f;
  uint8_t fault_mask = 0;
  uint32_t trigger_count = 0;
};

class ScannerController {
 public:
  void Begin();
  void Update();

  bool Home();
  bool MoveTo(float yawDeg, float pitchDeg);
  bool JogYaw(float deltaDeg);
  bool JogPitch(float deltaDeg);
  bool StartScan();
  bool PauseScan();
  bool ResumeScan();
  bool StopScan();
  bool Trigger();
  void Heartbeat();
  void EStop();
  bool ClearFault();

  ControllerStatus status() const;
  bool moving() const;
  float yaw_deg() const;
  float pitch_deg() const;
  float target_yaw_deg() const;
  float target_pitch_deg() const;
  MotionMode mode() const { return mode_; }
  uint8_t fault_mask() const { return fault_mask_; }

 private:
  struct AxisState {
    uint8_t step_pin = 0;
    uint8_t dir_pin = 0;
    float microsteps_per_degree = 1.0f;
    float min_deg = 0.0f;
    float max_deg = 0.0f;
    uint32_t step_interval_us = 2000;
    long current_steps = 0;
    long target_steps = 0;
    bool pulse_high = false;
    uint32_t pulse_low_deadline_us = 0;
    uint32_t next_step_due_us = 0;

    AxisState() = default;
    AxisState(uint8_t stepPin, uint8_t dirPin, float microstepsPerDegree, float minDeg, float maxDeg,
              uint32_t stepIntervalUs)
        : step_pin(stepPin),
          dir_pin(dirPin),
          microsteps_per_degree(microstepsPerDegree),
          min_deg(minDeg),
          max_deg(maxDeg),
          step_interval_us(stepIntervalUs) {}
  };

  static constexpr uint8_t kFaultEStop = 0x01;
  static constexpr uint8_t kFaultWatchdog = 0x02;

  bool MoveInternal(float yawDeg, float pitchDeg, MotionMode requestedMode);
  void StopMotionInternal();
  void SetDriverEnabled(bool enabled);
  void UpdateAxis(AxisState& axis, uint32_t nowMicros);
  static bool TimeReached(uint32_t nowMicros, uint32_t deadlineMicros);
  static float ClampFloat(float value, float minValue, float maxValue);
  static long DegreesToSteps(float degrees, float microstepsPerDegree);
  static float StepsToDegrees(long steps, float microstepsPerDegree);

  AxisState yaw_axis_{hw::kYawStepPin, hw::kYawDirPin, hw::kYawMicrostepsPerDegree, hw::kYawMinDeg,
                      hw::kYawMaxDeg,
                      static_cast<uint32_t>(1000000.0f /
                                            (hw::kYawMaxSpeedDegPerSec * hw::kYawMicrostepsPerDegree))};
  AxisState pitch_axis_{hw::kPitchStepPin, hw::kPitchDirPin, hw::kPitchMicrostepsPerDegree,
                        hw::kPitchMinDeg, hw::kPitchMaxDeg,
                        static_cast<uint32_t>(1000000.0f /
                                              (hw::kPitchMaxSpeedDegPerSec * hw::kPitchMicrostepsPerDegree))};
  MotionMode mode_ = MotionMode::kIdle;
  uint8_t fault_mask_ = 0;
  uint32_t trigger_count_ = 0;
  uint32_t trigger_pulse_until_us_ = 0;
  uint32_t last_host_seen_ms_ = 0;
};

}  // namespace scanner