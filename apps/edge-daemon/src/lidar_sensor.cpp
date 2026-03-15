#include "lidar_sensor.hpp"

#include <cmath>
#include <memory>

namespace edge {

bool MockLidarSensor::Initialize() { return true; }

double MockLidarSensor::ReadDistanceMeters(double yaw_deg, double pitch_deg) {
  const double yaw_wave = std::sin((yaw_deg + 60.0) * 0.045);
  const double pitch_wave = std::cos((pitch_deg + 20.0) * 0.08);
  return 5.0 + (yaw_wave * 0.8) + (pitch_wave * 0.5);
}

std::unique_ptr<LidarSensor> CreateLidarSensor(const Config& config) {
  (void)config;
  return std::make_unique<MockLidarSensor>();
}

}  // namespace edge
