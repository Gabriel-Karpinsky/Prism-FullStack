#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"

namespace edge {
namespace {

class MockGpioBackend final : public IGpioBackend {
 public:
  explicit MockGpioBackend(Config config) : config_(std::move(config)) {}

  bool Initialize(std::string&) override { return true; }
  void Shutdown() override {}

  void SetEnabled(bool asserted) override { enabled_ = asserted; }
  void SetStatusLed(bool on) override { led_ = on; }
  void SetAxisDirection(AxisId axis, bool forward) override {
    if (axis == AxisId::Yaw) yaw_dir_forward_ = forward;
    else pitch_dir_forward_ = forward;
  }

  bool RunMotionWaveform(const WaveformPlan& plan, std::string&) override {
    last_plan_pulses_ = plan.pulses.size();
    last_plan_yaw_steps_ = plan.yaw_steps_signed;
    last_plan_pitch_steps_ = plan.pitch_steps_signed;
    return true;
  }

  bool IsMotionBusy() const override { return false; }
  void AbortMotion() override {}
  void PulseTrigger(std::uint32_t) override { ++trigger_count_; }
  const char* Name() const override { return "mock-gpio"; }

 private:
  Config config_;
  std::atomic<bool> enabled_{false};
  std::atomic<bool> led_{false};
  std::atomic<bool> yaw_dir_forward_{true};
  std::atomic<bool> pitch_dir_forward_{true};
  std::atomic<std::size_t> last_plan_pulses_{0};
  std::atomic<int> last_plan_yaw_steps_{0};
  std::atomic<int> last_plan_pitch_steps_{0};
  std::atomic<std::uint32_t> trigger_count_{0};
};

}  // namespace

std::unique_ptr<IGpioBackend> CreateMockGpioBackend(const Config& config) {
  return std::make_unique<MockGpioBackend>(config);
}

}  // namespace edge
