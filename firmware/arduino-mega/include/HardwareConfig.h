#pragma once

#include <Arduino.h>

namespace scanner {
namespace hw {

constexpr uint8_t kYawStepPin = 2;
constexpr uint8_t kYawDirPin = 5;
constexpr uint8_t kPitchStepPin = 3;
constexpr uint8_t kPitchDirPin = 6;
constexpr uint8_t kDriverEnablePin = 8;
constexpr bool kEnableActiveLow = true;
constexpr uint8_t kTriggerPin = 22;
constexpr uint8_t kStatusLedPin = LED_BUILTIN;

// These defaults assume 1.8 degree steppers with the external driver already
// configured to 128 microsteps. Adjust gear ratios for your mechanics.
constexpr long kFullStepsPerRevolution = 200;
constexpr long kMicrosteps = 128;
constexpr float kYawGearRatio = 1.0f;
constexpr float kPitchGearRatio = 1.0f;
constexpr float kYawMicrostepsPerDegree = (kFullStepsPerRevolution * kMicrosteps * kYawGearRatio) / 360.0f;
constexpr float kPitchMicrostepsPerDegree = (kFullStepsPerRevolution * kMicrosteps * kPitchGearRatio) / 360.0f;

constexpr float kYawMinDeg = -60.0f;
constexpr float kYawMaxDeg = 60.0f;
constexpr float kPitchMinDeg = -20.0f;
constexpr float kPitchMaxDeg = 35.0f;

constexpr float kYawMaxSpeedDegPerSec = 18.0f;
constexpr float kPitchMaxSpeedDegPerSec = 12.0f;
constexpr uint32_t kStepPulseWidthMicros = 4;
constexpr uint32_t kGarminTriggerPulseMicros = 25;
constexpr uint32_t kHostWatchdogMs = 1500;

}  // namespace hw
}  // namespace scanner
