#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_config.hpp"

namespace edge {

enum class AxisId : std::uint8_t { Yaw = 0, Pitch = 1 };

struct StepPulse {
  std::uint32_t time_us;
  std::uint8_t axis_mask;  // bit 0 = yaw, bit 1 = pitch
};

struct WaveformPlan {
  std::vector<StepPulse> pulses;
  bool yaw_forward = true;
  bool pitch_forward = true;
  int yaw_steps_signed = 0;
  int pitch_steps_signed = 0;
};

class IGpioBackend {
 public:
  virtual ~IGpioBackend() = default;

  virtual bool Initialize(std::string& error) = 0;
  virtual void Shutdown() = 0;

  virtual void SetEnabled(bool asserted) = 0;
  virtual void SetStatusLed(bool on) = 0;
  virtual void SetAxisDirection(AxisId axis, bool forward) = 0;

  // Runs the motion waveform synchronously (blocks until complete or aborted).
  // Returns false with error populated on failure (e.g. pigpio resource exhaustion).
  virtual bool RunMotionWaveform(const WaveformPlan& plan, std::string& error) = 0;

  virtual bool IsMotionBusy() const = 0;
  virtual void AbortMotion() = 0;

  virtual void PulseTrigger(std::uint32_t microseconds) = 0;

  virtual const char* Name() const = 0;
};

std::unique_ptr<IGpioBackend> CreateGpioBackend(const Config& config);

}  // namespace edge
