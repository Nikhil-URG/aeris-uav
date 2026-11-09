#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/msg/laser_scan.hpp"

struct ScanAnalysis
{
  bool blocked{false};
  float min_range{std::numeric_limits<float>::infinity()};
  float min_bearing{0.0F};
};

constexpr float kPi = 3.14159265358979323846F;

inline float normalize_angle(float angle)
{
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

inline ScanAnalysis analyze_scan(
  const sensor_msgs::msg::LaserScan & scan,
  double forward_cone_rad,
  double threshold_m,
  double min_valid_range_m,
  double max_valid_range_m)
{
  ScanAnalysis result;
  const double half_cone = forward_cone_rad / 2.0;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < min_valid_range_m ||
      range > max_valid_range_m)
    {
      continue;
    }

    const float bearing = normalize_angle(
      static_cast<float>(scan.angle_min +
      static_cast<double>(i) * scan.angle_increment));

    if (std::abs(bearing) > half_cone) {
      continue;
    }

    if (range < result.min_range) {
      result.min_range = range;
      result.min_bearing = bearing;
    }
  }

  result.blocked = result.min_range < static_cast<float>(threshold_m);
  return result;
}
