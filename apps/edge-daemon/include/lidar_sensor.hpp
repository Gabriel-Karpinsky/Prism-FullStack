#pragma once

#include <memory>

#include "config.hpp"

namespace edge {

class LidarSensor {
 public:
  virtual ~LidarSensor() = default;
  virtual bool Initialize() = 0;
  virtual double ReadDistanceMeters(double yaw_deg, double pitch_deg) = 0;
  virtual const char* Name() const = 0;
};

class MockLidarSensor final : public LidarSensor {
 public:
  bool Initialize() override;
  double ReadDistanceMeters(double yaw_deg, double pitch_deg) override;
  const char* Name() const override { return "mock-lidar"; }
};

std::unique_ptr<LidarSensor> CreateLidarSensor(const Config& config);

}  // namespace edge
