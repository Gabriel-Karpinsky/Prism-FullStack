// PigpioGpioBackend — DMA-backed stepper waveform generator for Raspberry Pi.
//
// Only compiled when HAS_PIGPIO is defined (i.e. building on a Pi with
// libpigpio-dev installed). Elsewhere the factory falls through to the mock.

#if defined(HAS_PIGPIO)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pigpio.h>

#include "gpio_backend.hpp"
#include "hardware_config.hpp"

namespace edge {
namespace {

constexpr int kWaveMaxPulses = 11500;  // pigpio wave buffer headroom below 12000.
constexpr std::size_t kMaxWaveChunks = 8;  // chained-waveform ceiling (~92k pulses).

struct WaveEvent {
  std::uint32_t time_us;
  std::uint32_t gpio_on;
  std::uint32_t gpio_off;
};

class PigpioGpioBackend final : public IGpioBackend {
 public:
  explicit PigpioGpioBackend(Config config) : config_(std::move(config)) {}

  ~PigpioGpioBackend() override { Shutdown(); }

  bool Initialize(std::string& error) override {
    if (initialized_) return true;

    if (gpioInitialise() == PI_INIT_FAILED) {
      error = "pigpio gpioInitialise() failed (need root, /dev/gpiomem, and no conflicting DMA users)";
      return false;
    }

    yaw_step_mask_    = 1u << config_.gpio.yaw_step;
    pitch_step_mask_  = 1u << config_.gpio.pitch_step;
    yaw_dir_mask_     = 1u << config_.gpio.yaw_dir;
    pitch_dir_mask_   = 1u << config_.gpio.pitch_dir;
    enable_mask_      = 1u << config_.gpio.enable;
    trigger_mask_     = 1u << config_.gpio.lidar_trigger;
    status_led_mask_  = config_.gpio.status_led == 0 ? 0u : (1u << config_.gpio.status_led);

    for (unsigned pin : {config_.gpio.yaw_step, config_.gpio.yaw_dir,
                         config_.gpio.pitch_step, config_.gpio.pitch_dir,
                         config_.gpio.enable, config_.gpio.lidar_trigger}) {
      if (gpioSetMode(pin, PI_OUTPUT) != 0) {
        error = "gpioSetMode failed for pin " + std::to_string(pin);
        gpioTerminate();
        return false;
      }
    }
    if (status_led_mask_ != 0) gpioSetMode(config_.gpio.status_led, PI_OUTPUT);

    // Fail-safe initial state: driver disabled, trigger low, steps/dir deasserted.
    SetEnabled(false);
    WritePinRaw(config_.gpio.lidar_trigger, false);
    WriteAsserted(config_.gpio.yaw_step,   false, config_.gpio.step_active_low);
    WriteAsserted(config_.gpio.pitch_step, false, config_.gpio.step_active_low);
    WriteAsserted(config_.gpio.yaw_dir,    false, config_.gpio.dir_active_low);
    WriteAsserted(config_.gpio.pitch_dir,  false, config_.gpio.dir_active_low);
    if (status_led_mask_ != 0) WritePinRaw(config_.gpio.status_led, false);

    initialized_ = true;
    return true;
  }

  void Shutdown() override {
    if (!initialized_) return;
    AbortMotion();
    SetEnabled(false);
    gpioWaveClear();
    gpioTerminate();
    initialized_ = false;
  }

  void SetEnabled(bool asserted) override {
    WriteAsserted(config_.gpio.enable, asserted, config_.gpio.enable_active_low);
  }

  void SetStatusLed(bool on) override {
    if (status_led_mask_ == 0) return;
    gpioWrite(config_.gpio.status_led, on ? 1 : 0);
  }

  void SetAxisDirection(AxisId axis, bool forward) override {
    const unsigned pin = axis == AxisId::Yaw ? config_.gpio.yaw_dir : config_.gpio.pitch_dir;
    WriteAsserted(pin, forward, config_.gpio.dir_active_low);
  }

  bool RunMotionWaveform(const WaveformPlan& plan, std::string& error) override {
    if (!StartMotionWaveform(plan, error)) return false;
    FinishMotionWaveform();
    return true;
  }

  // Launch the waveform without waiting. Used directly by continuous sweeps;
  // RunMotionWaveform wraps Start + Finish for blocking point-to-point moves.
  bool StartMotionWaveform(const WaveformPlan& plan, std::string& error) override {
    if (!initialized_) { error = "backend not initialized"; return false; }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    for (int id : active_wave_ids_) gpioWaveDelete(id);  // reap any stragglers
    active_wave_ids_.clear();
    abort_requested_.store(false);
    busy_.store(false);
    if (plan.pulses.empty()) return true;

    SetAxisDirection(AxisId::Yaw,   plan.yaw_forward);
    SetAxisDirection(AxisId::Pitch, plan.pitch_forward);
    gpioDelay(5);  // let DIR settle before first STEP pulse (datasheet spec).

    auto events = BuildEvents(plan);
    if (events.empty()) return true;

    auto pulses = EventsToPigpioPulses(events);

    // pigpio caps a single waveform at ~12000 pulses. A long move (a full-travel
    // seek, or a whole continuous-scan row) is split into chunks of
    // kWaveMaxPulses; gpioWaveChain then transmits them seamlessly so the motor
    // sees one continuous trapezoidal profile with no mid-move pause. usDelay
    // was computed from the absolute event timeline, so the delay at a chunk
    // boundary already carries the gap to the next chunk.
    gpioWaveClear();
    std::vector<int> wave_ids;
    for (std::size_t off = 0; off < pulses.size(); off += kWaveMaxPulses) {
      const unsigned n = static_cast<unsigned>(
          std::min<std::size_t>(kWaveMaxPulses, pulses.size() - off));
      if (gpioWaveAddGeneric(n, pulses.data() + off) < 0 ||
          wave_ids.size() >= kMaxWaveChunks) {
        for (int id : wave_ids) gpioWaveDelete(id);
        error = "move too large for pigpio wave memory; reduce travel distance";
        return false;
      }
      const int id = gpioWaveCreate();
      if (id < 0) {
        for (int existing : wave_ids) gpioWaveDelete(existing);
        error = "gpioWaveCreate failed (out of pigpio wave memory)";
        return false;
      }
      wave_ids.push_back(id);
    }
    if (wave_ids.empty()) return true;

    // gpioWaveChain takes wave ids as raw bytes; ids are small integers.
    std::vector<char> chain;
    chain.reserve(wave_ids.size());
    for (int id : wave_ids) chain.push_back(static_cast<char>(id));

    busy_.store(true);
    if (gpioWaveChain(chain.data(), static_cast<int>(chain.size())) < 0) {
      busy_.store(false);
      for (int id : wave_ids) gpioWaveDelete(id);
      error = "gpioWaveChain failed";
      return false;
    }
    active_wave_ids_ = std::move(wave_ids);
    return true;
  }

  // Wait for the active waveform to drain (respecting aborts) and release the
  // DMA wave buffers. Safe to call when nothing is in flight.
  void FinishMotionWaveform() override {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    while (gpioWaveTxBusy()) {
      if (abort_requested_.load()) {
        gpioWaveTxStop();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (int id : active_wave_ids_) gpioWaveDelete(id);
    active_wave_ids_.clear();
    busy_.store(false);
  }

  // Reflects the real DMA transmitter so a sweep's sampling loop can detect
  // completion (busy_ alone would stay set until FinishMotionWaveform).
  bool IsMotionBusy() const override { return initialized_ && gpioWaveTxBusy() != 0; }

  void AbortMotion() override {
    if (!initialized_) return;
    abort_requested_.store(true);
    if (gpioWaveTxBusy()) gpioWaveTxStop();
  }

  void PulseTrigger(std::uint32_t microseconds) override {
    if (!initialized_ || microseconds == 0) return;
    gpioTrigger(config_.gpio.lidar_trigger, microseconds, 1);
  }

  const char* Name() const override { return "pigpio"; }

 private:
  static int LevelFor(bool asserted, bool active_low) {
    return (asserted != active_low) ? 1 : 0;
  }
  static void WriteAsserted(unsigned pin, bool asserted, bool active_low) {
    gpioWrite(pin, LevelFor(asserted, active_low));
  }
  static void WritePinRaw(unsigned pin, bool level) { gpioWrite(pin, level ? 1 : 0); }

  std::vector<WaveEvent> BuildEvents(const WaveformPlan& plan) const {
    std::vector<WaveEvent> events;
    events.reserve(plan.pulses.size() * 2);

    const bool step_al = config_.gpio.step_active_low;
    const std::uint32_t pulse_width_us = static_cast<std::uint32_t>(std::max(1, config_.safety.step_pulse_us));

    for (const auto& p : plan.pulses) {
      std::uint32_t mask = 0;
      if (p.axis_mask & 0b01) mask |= yaw_step_mask_;
      if (p.axis_mask & 0b10) mask |= pitch_step_mask_;
      if (mask == 0) continue;

      // Assert (start of pulse):
      WaveEvent assert_ev{};
      assert_ev.time_us = p.time_us;
      if (step_al) assert_ev.gpio_off = mask;
      else         assert_ev.gpio_on  = mask;
      events.push_back(assert_ev);

      // Deassert (end of pulse):
      WaveEvent deassert_ev{};
      deassert_ev.time_us = p.time_us + pulse_width_us;
      if (step_al) deassert_ev.gpio_on  = mask;
      else         deassert_ev.gpio_off = mask;
      events.push_back(deassert_ev);
    }

    std::sort(events.begin(), events.end(),
              [](const WaveEvent& a, const WaveEvent& b) { return a.time_us < b.time_us; });

    // Merge events at identical timestamps (multi-axis coincident edges).
    std::vector<WaveEvent> merged;
    merged.reserve(events.size());
    for (const auto& ev : events) {
      if (!merged.empty() && merged.back().time_us == ev.time_us) {
        merged.back().gpio_on  |= ev.gpio_on;
        merged.back().gpio_off |= ev.gpio_off;
      } else {
        merged.push_back(ev);
      }
    }
    return merged;
  }

  std::vector<gpioPulse_t> EventsToPigpioPulses(const std::vector<WaveEvent>& events) const {
    std::vector<gpioPulse_t> out;
    out.reserve(events.size());
    for (std::size_t i = 0; i < events.size(); ++i) {
      const auto& ev = events[i];
      gpioPulse_t p{};
      p.gpioOn  = ev.gpio_on;
      p.gpioOff = ev.gpio_off;
      p.usDelay = (i + 1 < events.size())
                      ? (events[i + 1].time_us - ev.time_us)
                      : 5u;  // final short tail so the last edge actually transmits.
      out.push_back(p);
    }
    return out;
  }

  Config config_;
  bool initialized_ = false;
  std::uint32_t yaw_step_mask_ = 0;
  std::uint32_t pitch_step_mask_ = 0;
  std::uint32_t yaw_dir_mask_ = 0;
  std::uint32_t pitch_dir_mask_ = 0;
  std::uint32_t enable_mask_ = 0;
  std::uint32_t trigger_mask_ = 0;
  std::uint32_t status_led_mask_ = 0;

  std::mutex motion_mutex_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> abort_requested_{false};
  std::vector<int> active_wave_ids_;  // waves queued by StartMotionWaveform, freed by Finish
};

}  // namespace

std::unique_ptr<IGpioBackend> CreatePigpioGpioBackend(const Config& config) {
  return std::make_unique<PigpioGpioBackend>(config);
}

}  // namespace edge

#endif  // HAS_PIGPIO
