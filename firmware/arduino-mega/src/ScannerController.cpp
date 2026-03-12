#include "ScannerController.h"

namespace scanner {

namespace {
constexpr float kYawMinDeg = -60.0f;
constexpr float kYawMaxDeg = 60.0f;
constexpr float kPitchMinDeg = -20.0f;
constexpr float kPitchMaxDeg = 35.0f;
constexpr uint8_t kFaultEStop = 0x01;
}  // namespace

void ScannerController::Begin() {
  yaw_deg_ = 0.0f;
  pitch_deg_ = 0.0f;
  mode_ = MotionMode::kIdle;
  fault_mask_ = 0;
}

void ScannerController::Update() {
  // Replace this with timer-driven step generation, limit switch sampling,
  // and watchdog checks in the real firmware.
}

void ScannerController::Home() {
  if (mode_ == MotionMode::kFault) {
    return;
  }

  yaw_deg_ = 0.0f;
  pitch_deg_ = 0.0f;
  mode_ = MotionMode::kIdle;
}

void ScannerController::Jog(float yawDeltaDeg, float pitchDeltaDeg) {
  if (mode_ == MotionMode::kFault) {
    return;
  }

  yaw_deg_ += yawDeltaDeg;
  pitch_deg_ += pitchDeltaDeg;
  ClampPosition();
  mode_ = MotionMode::kManual;
}

void ScannerController::StartScan() {
  if (mode_ == MotionMode::kFault) {
    return;
  }
  mode_ = MotionMode::kScanning;
}

void ScannerController::PauseScan() {
  if (mode_ == MotionMode::kScanning) {
    mode_ = MotionMode::kPaused;
  }
}

void ScannerController::StopScan() {
  if (mode_ != MotionMode::kFault) {
    mode_ = MotionMode::kIdle;
  }
}

void ScannerController::EStop() {
  mode_ = MotionMode::kFault;
  fault_mask_ |= kFaultEStop;
}

void ScannerController::ClearFault() {
  fault_mask_ = 0;
  mode_ = MotionMode::kIdle;
}

void ScannerController::ClampPosition() {
  if (yaw_deg_ < kYawMinDeg) yaw_deg_ = kYawMinDeg;
  if (yaw_deg_ > kYawMaxDeg) yaw_deg_ = kYawMaxDeg;
  if (pitch_deg_ < kPitchMinDeg) pitch_deg_ = kPitchMinDeg;
  if (pitch_deg_ > kPitchMaxDeg) pitch_deg_ = kPitchMaxDeg;
}

}  // namespace scanner
