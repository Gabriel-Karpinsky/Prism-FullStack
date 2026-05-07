#include "lidar_sensor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <thread>
#include <utility>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace edge {

bool MockLidarSensor::Initialize() {
  last_error_.clear();
  return true;
}

double MockLidarSensor::ReadDistanceMeters(double yaw_deg, double pitch_deg) {
  const double yaw_wave = std::sin((yaw_deg + 60.0) * 0.045);
  const double pitch_wave = std::cos((pitch_deg + 20.0) * 0.08);
  return 5.0 + (yaw_wave * 0.8) + (pitch_wave * 0.5);
}

GarminLidarLiteV3HPSensor::GarminLidarLiteV3HPSensor(LidarHardwareConfig config)
    : config_(std::move(config)) {}

GarminLidarLiteV3HPSensor::~GarminLidarLiteV3HPSensor() { Close(); }

bool GarminLidarLiteV3HPSensor::Initialize() {
#ifdef __linux__
  const std::string device = "/dev/i2c-" + std::to_string(config_.i2c_bus);
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_error_ = std::string("failed to open ") + device + ": " + std::strerror(errno);
    return false;
  }
  if (ioctl(fd_, I2C_SLAVE, config_.i2c_address) < 0) {
    last_error_ = std::string("failed to select lidar address: ") + std::strerror(errno);
    Close();
    return false;
  }
  last_error_.clear();
  return true;
#else
  last_error_ = "garmin lidar transport is only implemented for Linux hosts";
  return false;
#endif
}

double GarminLidarLiteV3HPSensor::ReadDistanceMeters(double, double) {
#ifdef __linux__
  if (fd_ < 0 && !Initialize()) return std::numeric_limits<double>::quiet_NaN();
  if (!WriteRegister(0x00, 0x01)) return std::numeric_limits<double>::quiet_NaN();
  if (!WaitForReady())            return std::numeric_limits<double>::quiet_NaN();

  std::uint16_t distance_cm = 0;
  if (!ReadRegister16(0x0f, distance_cm)) return std::numeric_limits<double>::quiet_NaN();

  last_error_.clear();
  return static_cast<double>(distance_cm) / 100.0;
#else
  return std::numeric_limits<double>::quiet_NaN();
#endif
}

bool GarminLidarLiteV3HPSensor::WriteRegister(std::uint8_t reg, std::uint8_t value) {
#ifdef __linux__
  const std::uint8_t buffer[2] = {reg, value};
  if (write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
    last_error_ = std::string("i2c write failed: ") + std::strerror(errno);
    return false;
  }
  return true;
#else
  (void)reg; (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::ReadRegister(std::uint8_t reg, std::uint8_t& value) {
#ifdef __linux__
  if (write(fd_, &reg, 1) != 1) {
    last_error_ = std::string("i2c register select failed: ") + std::strerror(errno);
    return false;
  }
  if (read(fd_, &value, 1) != 1) {
    last_error_ = std::string("i2c register read failed: ") + std::strerror(errno);
    return false;
  }
  return true;
#else
  (void)reg; (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::ReadRegister16(std::uint8_t reg, std::uint16_t& value) {
#ifdef __linux__
  if (write(fd_, &reg, 1) != 1) {
    last_error_ = std::string("i2c register select failed: ") + std::strerror(errno);
    return false;
  }
  std::uint8_t bytes[2] = {0, 0};
  if (read(fd_, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes))) {
    last_error_ = std::string("i2c register read failed: ") + std::strerror(errno);
    return false;
  }
  value = static_cast<std::uint16_t>((static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
  return true;
#else
  (void)reg; (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::WaitForReady() {
#ifdef __linux__
  // The v3HP can take several hundred ms to complete a measurement when the
  // target is dim, far, or out of range — the laser keeps integrating until
  // it has signal. Garmin's reference Arduino library caps with a 10 000-
  // iteration counter (≈500 ms at 100 kHz). 500 ms matches that intent.
  //
  // Status register layout (0x01):
  //   bit 0 = system busy        ← we wait for this to clear
  //   bit 1 = signal not valid   ← treat as "measurement complete but no return"
  //   bit 2 = reference overflow
  //   bit 3 = signal overflow
  //   bit 4 = system failure     ← treat as hard fault
  //   bit 6 = process done
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  std::uint8_t last_status = 0xFF;
  while (std::chrono::steady_clock::now() < deadline) {
    std::uint8_t status = 0;
    if (!ReadRegister(0x01, status)) return false;
    last_status = status;

    // 0xFF on every read = bus electrical issue (LIDAR not really driving SDA);
    // bail out quickly so the operator sees a useful error instead of a 500 ms
    // hang every cycle.
    if (status == 0xFFU) {
      last_error_ = "lidar status reads 0xFF — bus electrical issue (check decoupling cap, pull-ups, ground)";
      return false;
    }

    if ((status & 0x10U) != 0U) {
      last_error_ = "lidar reports system failure (status bit 4 set)";
      return false;
    }

    if ((status & 0x01U) == 0U) return true;  // not busy → done
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf),
                "lidar measurement timed out (last status=0x%02X)",
                last_status);
  last_error_ = buf;
  return false;
#else
  return false;
#endif
}

void GarminLidarLiteV3HPSensor::Close() {
#ifdef __linux__
  if (fd_ >= 0) { close(fd_); fd_ = -1; }
#endif
}

std::unique_ptr<LidarSensor> CreateLidarSensor(const Config& config) {
  if (config.lidar.simulate || config.simulate_hardware) {
    return std::make_unique<MockLidarSensor>();
  }
  return std::make_unique<GarminLidarLiteV3HPSensor>(config.lidar);
}

}  // namespace edge
