#include "gpio_backend.hpp"

#include <memory>

#include "hardware_config.hpp"

namespace edge {

std::unique_ptr<IGpioBackend> CreateMockGpioBackend(const Config& config);
#if defined(HAS_PIGPIO)
std::unique_ptr<IGpioBackend> CreatePigpioGpioBackend(const Config& config);
#endif

std::unique_ptr<IGpioBackend> CreateGpioBackend(const Config& config) {
#if defined(HAS_PIGPIO)
  if (!config.simulate_hardware) {
    return CreatePigpioGpioBackend(config);
  }
#endif
  return CreateMockGpioBackend(config);
}

}  // namespace edge
