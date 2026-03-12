#pragma once

#include <Arduino.h>

namespace scanner {

enum class MotionMode : uint8_t {
  kIdle = 0,
  kManual = 1,
  kScanning = 2,
  kPaused = 3,
  kFault = 4,
};

class ScannerController {
 public:
  void Begin();
  void Update();

  void Home();
  void Jog(float yawDeltaDeg, float pitchDeltaDeg);
  void StartScan();
  void PauseScan();
  void StopScan();
  void EStop();
  void ClearFault();

  float yaw_deg() const { return yaw_deg_; }
  float pitch_deg() const { return pitch_deg_; }
  MotionMode mode() const { return mode_; }
  uint8_t fault_mask() const { return fault_mask_; }

 private:
  void ClampPosition();

  float yaw_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;
  MotionMode mode_ = MotionMode::kIdle;
  uint8_t fault_mask_ = 0;
};

}  // namespace scanner
