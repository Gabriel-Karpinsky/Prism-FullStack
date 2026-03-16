#include "lidar_sensor.hpp"

#include <chrono>
#include <cmath>
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

GarminLidarLiteV3HPSensor::GarminLidarLiteV3HPSensor(Config config) : config_(std::move(config)) {}

GarminLidarLiteV3HPSensor::~GarminLidarLiteV3HPSensor() { Close(); }

bool GarminLidarLiteV3HPSensor::Initialize() {
#ifdef __linux__
  const std::string device = "/dev/i2c-" + std::to_string(config_.lidar_bus);
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_error_ = std::string("failed to open ") + device + ": " + std::strerror(errno);
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, config_.lidar_address) < 0) {
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

double GarminLidarLiteV3HPSensor::ReadDistanceMeters(double yaw_deg, double pitch_deg) {
  (void)yaw_deg;
  (void)pitch_deg;
#ifdef __linux__
  if (fd_ < 0 && !Initialize()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (!WriteRegister(0x00, 0x01)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!WaitForReady()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  uint16_t distance_cm = 0;
  if (!ReadRegister16(0x0f, distance_cm)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  last_error_.clear();
  return static_cast<double>(distance_cm) / 100.0;
#else
  return std::numeric_limits<double>::quiet_NaN();
#endif
}

bool GarminLidarLiteV3HPSensor::WriteRegister(uint8_t reg, uint8_t value) {
#ifdef __linux__
  const uint8_t buffer[2] = {reg, value};
  if (write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
    last_error_ = std::string("i2c write failed: ") + std::strerror(errno);
    return false;
  }
  return true;
#else
  (void)reg;
  (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::ReadRegister(uint8_t reg, uint8_t& value) {
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
  (void)reg;
  (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::ReadRegister16(uint8_t reg, uint16_t& value) {
#ifdef __linux__
  if (write(fd_, &reg, 1) != 1) {
    last_error_ = std::string("i2c register select failed: ") + std::strerror(errno);
    return false;
  }

  uint8_t bytes[2] = {0, 0};
  if (read(fd_, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes))) {
    last_error_ = std::string("i2c register read failed: ") + std::strerror(errno);
    return false;
  }

  value = static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8U) | bytes[1]);
  return true;
#else
  (void)reg;
  (void)value;
  return false;
#endif
}

bool GarminLidarLiteV3HPSensor::WaitForReady() {
#ifdef __linux__
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(120);
  while (std::chrono::steady_clock::now() < deadline) {
    uint8_t status = 0;
    if (!ReadRegister(0x01, status)) {
      return false;
    }
    if ((status & 0x01U) == 0U) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  last_error_ = "lidar measurement timed out";
  return false;
#else
  return false;
#endif
}

void GarminLidarLiteV3HPSensor::Close() {
#ifdef __linux__
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
#endif
}

std::unique_ptr<LidarSensor> CreateLidarSensor(const Config& config) {
  if (config.simulate_lidar || config.simulate_hardware) {
    return std::make_unique<MockLidarSensor>();
  }
  return std::make_unique<GarminLidarLiteV3HPSensor>(config);
}

}  // namespace edge
