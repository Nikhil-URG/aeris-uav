#include <cmath>
#include <limits>
#include <vector>

#include "aeris_navigation/scan_analysis.hpp"
#include "gtest/gtest.h"

namespace
{
sensor_msgs::msg::LaserScan make_scan(float angle_min, float angle_increment,
  const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.range_min = 0.1F;
  scan.range_max = 30.0F;
  scan.ranges = ranges;
  return scan;
}
}  // namespace

TEST(ScanAnalysis, DetectsObstacleInForwardCone)
{
  const float inf = std::numeric_limits<float>::infinity();
  auto scan = make_scan(-1.0F, 0.1F, std::vector<float>(21, inf));
  scan.ranges[10] = 2.0F;  // bearing ~0.0 rad

  const auto result = analyze_scan(scan, 1.4, 3.0, 0.2, 20.0);

  EXPECT_TRUE(result.blocked);
  EXPECT_FLOAT_EQ(result.min_range, 2.0F);
  EXPECT_NEAR(result.min_bearing, 0.0F, 1e-5F);
}

TEST(ScanAnalysis, IgnoresOutOfConeObstacle)
{
  const float inf = std::numeric_limits<float>::infinity();
  auto scan = make_scan(-1.0F, 0.1F, std::vector<float>(21, inf));
  scan.ranges[0] = 1.0F;  // bearing -1.0 rad, outside a 1.4 rad cone

  const auto result = analyze_scan(scan, 1.4, 3.0, 0.2, 20.0);

  EXPECT_FALSE(result.blocked);
  EXPECT_FALSE(std::isfinite(result.min_range));
}

TEST(ScanAnalysis, ClearSceneIsNotBlocked)
{
  const float inf = std::numeric_limits<float>::infinity();
  auto scan = make_scan(-1.0F, 0.1F, std::vector<float>(21, inf));

  const auto result = analyze_scan(scan, 1.4, 3.0, 0.2, 20.0);

  EXPECT_FALSE(result.blocked);
}

TEST(ScanAnalysis, IgnoresInvalidRanges)
{
  auto scan = make_scan(-0.5F, 0.1F, {0.0F, 1.0F, 0.05F, 2.0F});
  scan.ranges[3] = std::numeric_limits<float>::quiet_NaN();

  const auto result = analyze_scan(scan, 1.4, 3.0, 0.2, 20.0);

  // The 0.0 m and 0.05 m beams are below the valid range, the 2.0 m beam is
  // NaN, and the 1.0 m beam remains and blocks.
  EXPECT_TRUE(result.blocked);
  EXPECT_FLOAT_EQ(result.min_range, 1.0F);
}
