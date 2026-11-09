#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "aeris_interfaces/msg/obstacle.hpp"
#include "aeris_interfaces/msg/obstacle_array.hpp"
#include "aeris_navigation/scan_analysis.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("aeris_navigation")
  {
    scan_topic_ = this->declare_parameter<std::string>(
      "scan_topic", "/aeris/lidar/scan");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/aeris/navigation/obstacles");
    forward_cone_deg_ = this->declare_parameter<double>(
      "forward_cone_deg", 70.0);
    obstacle_threshold_m_ = this->declare_parameter<double>(
      "obstacle_threshold_m", 3.0);
    min_valid_range_m_ = this->declare_parameter<double>(
      "min_valid_range_m", 0.2);
    max_valid_range_m_ = this->declare_parameter<double>(
      "max_valid_range_m", 20.0);
    publish_max_range_m_ = this->declare_parameter<double>(
      "publish_max_range_m", 10.0);
    sector_width_deg_ = this->declare_parameter<double>(
      "sector_width_deg", 10.0);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavigationNode::scan_callback, this, std::placeholders::_1));

    obstacle_pub_ =
      this->create_publisher<aeris_interfaces::msg::ObstacleArray>(
        output_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "AERIS navigation initialized: scan='%s', output='%s', "
      "forward_cone=%.1f deg, threshold=%.1f m.",
      scan_topic_.c_str(), output_topic_.c_str(),
      forward_cone_deg_, obstacle_threshold_m_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double cone_rad = forward_cone_deg_ * 3.14159265358979323846 / 180.0;
    const ScanAnalysis analysis = analyze_scan(
      *msg, cone_rad, obstacle_threshold_m_,
      min_valid_range_m_, max_valid_range_m_);

    aeris_interfaces::msg::ObstacleArray output;
    output.stamp = msg->header.stamp;
    output.frame_id = msg->header.frame_id;
    output.blocked = analysis.blocked;
    output.forward_min_range_m = analysis.min_range;
    output.forward_min_bearing_rad = analysis.min_bearing;

    // Publish a coarse obstacle map across the whole scan. The mission
    // manager evaluates the corridor toward its current target rather than
    // assuming the target is straight ahead.
    const double sector_width =
      std::max(1.0, sector_width_deg_) * 3.14159265358979323846 / 180.0;
    if (sector_width > 0.0 && msg->angle_increment > 0.0F) {
      const int sector_count = static_cast<int>(
        std::ceil((msg->angle_max - msg->angle_min) / sector_width));
      std::vector<float> sector_min(
        static_cast<size_t>(std::max(1, sector_count)),
        std::numeric_limits<float>::infinity());
      std::vector<float> sector_bearing(
        static_cast<size_t>(std::max(1, sector_count)), 0.0F);

      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float range = msg->ranges[i];
        if (!std::isfinite(range) || range < min_valid_range_m_ ||
          range > std::min(max_valid_range_m_, publish_max_range_m_))
        {
          continue;
        }

        const float bearing = static_cast<float>(
          msg->angle_min + static_cast<double>(i) * msg->angle_increment);
        const int index = static_cast<int>(
          std::floor((bearing - msg->angle_min) / sector_width));
        if (index < 0 || index >= sector_count) {
          continue;
        }
        if (range < sector_min[static_cast<size_t>(index)]) {
          sector_min[static_cast<size_t>(index)] = range;
          sector_bearing[static_cast<size_t>(index)] = bearing;
        }
      }

      for (size_t i = 0; i < sector_min.size(); ++i) {
        if (!std::isfinite(sector_min[i])) {
          continue;
        }
        aeris_interfaces::msg::Obstacle obstacle;
        obstacle.bearing_rad = sector_bearing[i];
        obstacle.range_m = sector_min[i];
        output.obstacles.push_back(obstacle);
      }
    }

    obstacle_pub_->publish(output);

    if (analysis.blocked) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Obstacle ahead: range=%.2f m bearing=%.2f rad (blocked).",
        analysis.min_range, analysis.min_bearing);
    }
  }

  std::string scan_topic_;
  std::string output_topic_;
  double forward_cone_deg_{70.0};
  double obstacle_threshold_m_{3.0};
  double min_valid_range_m_{0.2};
  double max_valid_range_m_{20.0};
  double publish_max_range_m_{10.0};
  double sector_width_deg_{10.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<aeris_interfaces::msg::ObstacleArray>::SharedPtr
    obstacle_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
