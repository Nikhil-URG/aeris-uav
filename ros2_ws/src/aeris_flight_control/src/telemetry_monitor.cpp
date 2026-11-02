#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"

class TelemetryMonitor : public rclcpp::Node
{
public:
  TelemetryMonitor()
  : Node("aeris_telemetry_monitor")
  {
    position_topic_ = this->declare_parameter<std::string>(
      "position_topic", "/fmu/out/vehicle_local_position");
    log_every_n_ = this->declare_parameter<int>("log_every_n", 50);
    moving_speed_threshold_ =
      this->declare_parameter<double>("moving_speed_threshold", 0.1);

    if (log_every_n_ < 1) {
      RCLCPP_WARN(
        this->get_logger(),
        "log_every_n=%d is invalid; clamping to 1.", log_every_n_);
      log_every_n_ = 1;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    subscription_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        position_topic_,
        qos,
        std::bind(
          &TelemetryMonitor::position_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "AERIS telemetry monitor initialized "
      "(topic='%s', log_every_n=%d, moving_speed_threshold=%.2f m/s).",
      position_topic_.c_str(),
      log_every_n_,
      moving_speed_threshold_);
  }

private:
  void position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    ++counter_;

    if (counter_ % static_cast<uint64_t>(log_every_n_) != 0) {
      return;
    }

    const bool position_valid = msg->xy_valid && msg->z_valid;
    const bool velocity_valid = msg->v_xy_valid && msg->v_z_valid;
    const double speed =
      std::sqrt(msg->vx * msg->vx + msg->vy * msg->vy + msg->vz * msg->vz);
    const bool moving = velocity_valid && speed > moving_speed_threshold_;

    RCLCPP_INFO(
      this->get_logger(),
      "Position [NED] X=%.2f Y=%.2f Z=%.2f | "
      "Velocity [NED] VX=%.2f VY=%.2f VZ=%.2f | "
      "Speed=%.2f m/s | PosValid=%s VelValid=%s | Moving=%s",
      msg->x,
      msg->y,
      msg->z,
      msg->vx,
      msg->vy,
      msg->vz,
      speed,
      position_valid ? "YES" : "NO",
      velocity_valid ? "YES" : "NO",
      moving ? "YES" : "NO");
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    subscription_;

  std::string position_topic_;
  int log_every_n_{50};
  double moving_speed_threshold_{0.1};
  uint64_t counter_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryMonitor>());
  rclcpp::shutdown();
  return 0;
}
