#pragma once

#include <memory>
#include <string>

#include "config.hpp"

namespace edge {

class LidarSensor {
 public:
  virtual ~LidarSensor() = default;
  virtual bool Initialize() = 0;
  virtual double ReadDistanceMeters(double yaw_deg, double pitch_deg) = 0;
  virtual const char* Name() const = 0;
  virtual const std::string& last_error() const = 0;
};

class MockLidarSensor final : public LidarSensor {
 public:
  bool Initialize() override;
  double ReadDistanceMeters(double yaw_deg, double pitch_deg) override;
  const char* Name() const override { return "mock-lidar"; }
  const std::string& last_error() const override { return last_error_; }

 private:
  std::string last_error_;
};

class GarminLidarLiteV3HPSensor final : public LidarSensor {
 public:
  explicit GarminLidarLiteV3HPSensor(Config config);
  ~GarminLidarLiteV3HPSensor() override;

  bool Initialize() override;
  double ReadDistanceMeters(double yaw_deg, double pitch_deg) override;
  const char* Name() const override { return "garmin-lidar-lite-v3hp"; }
  const std::string& last_error() const override { return last_error_; }

 private:
  bool WriteRegister(uint8_t reg, uint8_t value);
  bool ReadRegister(uint8_t reg, uint8_t& value);
  bool ReadRegister16(uint8_t reg, uint16_t& value);
  bool WaitForReady();
  void Close();

  Config config_;
  std::string last_error_;
#ifdef __linux__
  int fd_ = -1;
#endif
};

std::unique_ptr<LidarSensor> CreateLidarSensor(const Config& config);

}  // namespace edge
