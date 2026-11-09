#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "aeris_interfaces/msg/mission_status.hpp"
#include "aeris_interfaces/msg/safety_status.hpp"
#include "aeris_interfaces/srv/inject_fault.hpp"
#include "aeris_safety/safety_evaluator.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode()
  : Node("aeris_safety")
  {
    pose_topic_ = this->declare_parameter<std::string>(
      "pose_topic", "/fmu/out/vehicle_local_position_v1");
    battery_topic_ = this->declare_parameter<std::string>(
      "battery_topic", "/fmu/out/battery_status_v1");
    vehicle_status_topic_ = this->declare_parameter<std::string>(
      "vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    control_mode_topic_ = this->declare_parameter<std::string>(
      "control_mode_topic", "/fmu/out/vehicle_control_mode");
    mission_status_topic_ = this->declare_parameter<std::string>(
      "mission_status_topic", "/aeris_mission_manager/status");
    status_topic_ = this->declare_parameter<std::string>(
      "status_topic", "/aeris/safety/status");

    land_service_name_ = this->declare_parameter<std::string>(
      "land_service", "/aeris_offboard_controller/land");
    rtl_service_name_ = this->declare_parameter<std::string>(
      "rtl_service", "/aeris_offboard_controller/rtl");
    hold_service_name_ = this->declare_parameter<std::string>(
      "hold_service", "/aeris_offboard_controller/hold");

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    action_retry_interval_s_ =
      this->declare_parameter<double>("action_retry_interval_s", 2.0);

    config_.pose_timeout_s =
      this->declare_parameter<double>("pose_timeout_s", 1.0);
    config_.battery_min_remaining =
      this->declare_parameter<double>("battery_min_remaining", 0.25);
    config_.geofence_radius_m =
      this->declare_parameter<double>("geofence_radius_m", 20.0);
    config_.mission_heartbeat_timeout_s =
      this->declare_parameter<double>("mission_heartbeat_timeout_s", 3.0);
    config_.require_mission_heartbeat =
      this->declare_parameter<bool>("require_mission_heartbeat", true);
    config_.action_for_stale_pose =
      this->declare_parameter<std::string>("action_for_stale_pose", "land");
    config_.action_for_low_battery =
      this->declare_parameter<std::string>("action_for_low_battery", "rtl");
    config_.action_for_geofence_breach =
      this->declare_parameter<std::string>("action_for_geofence_breach", "rtl");
    config_.action_for_mission_heartbeat_loss =
      this->declare_parameter<std::string>(
        "action_for_mission_heartbeat_loss", "rtl");
    geofence_center_x_ =
      this->declare_parameter<double>("geofence_center_x", 0.0);
    geofence_center_y_ =
      this->declare_parameter<double>("geofence_center_y", 0.0);

    publish_rate_hz_ = std::clamp(publish_rate_hz_, 1.0, 50.0);
    action_retry_interval_s_ =
      std::clamp(action_retry_interval_s_, 0.2, 30.0);

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      pose_topic_, px4_qos,
      std::bind(&SafetyNode::on_position, this, std::placeholders::_1));
    battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
      battery_topic_, px4_qos,
      std::bind(&SafetyNode::on_battery, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, px4_qos,
      std::bind(&SafetyNode::on_vehicle_status, this, std::placeholders::_1));
    control_mode_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleControlMode>(
      control_mode_topic_, px4_qos,
      std::bind(&SafetyNode::on_control_mode, this, std::placeholders::_1));
    mission_status_sub_ =
      this->create_subscription<aeris_interfaces::msg::MissionStatus>(
      mission_status_topic_, 10,
      std::bind(&SafetyNode::on_mission_status, this, std::placeholders::_1));

    status_pub_ =
      this->create_publisher<aeris_interfaces::msg::SafetyStatus>(
      status_topic_, 10);

    inject_service_ =
      this->create_service<aeris_interfaces::srv::InjectFault>(
      "~/inject_fault",
      std::bind(&SafetyNode::on_inject_fault, this,
        std::placeholders::_1, std::placeholders::_2));
    clear_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/clear_faults",
      std::bind(&SafetyNode::on_clear_faults, this,
        std::placeholders::_1, std::placeholders::_2));

    land_client_ = this->create_client<std_srvs::srv::Trigger>(
      land_service_name_);
    rtl_client_ = this->create_client<std_srvs::srv::Trigger>(
      rtl_service_name_);
    hold_client_ = this->create_client<std_srvs::srv::Trigger>(
      hold_service_name_);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(
      period, std::bind(&SafetyNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "AERIS safety initialized: status='%s', pose_timeout=%.1f s, "
      "battery_min=%.2f, geofence=%.1f m, heartbeat_timeout=%.1f s.",
      status_topic_.c_str(), config_.pose_timeout_s,
      config_.battery_min_remaining, config_.geofence_radius_m,
      config_.mission_heartbeat_timeout_s);
    RCLCPP_INFO(
      get_logger(),
      "Services: %s/inject_fault, %s/clear_faults",
      get_name(), get_name());
  }

private:
  static double age_s(
    const std::chrono::steady_clock::time_point & then,
    const std::chrono::steady_clock::time_point & now)
  {
    return std::chrono::duration<double>(now - then).count();
  }

  void on_position(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    position_ = *msg;
    have_position_ = true;
    last_position_time_ = std::chrono::steady_clock::now();
  }

  void on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    battery_ = *msg;
    have_battery_ = true;
  }

  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
    have_vehicle_status_ = true;
  }

  void on_control_mode(
    const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    control_mode_ = *msg;
    have_control_mode_ = true;
  }

  void on_mission_status(
    const aeris_interfaces::msg::MissionStatus::SharedPtr msg)
  {
    mission_active_ = msg->active;
    have_mission_status_ = true;
    last_mission_status_time_ = std::chrono::steady_clock::now();
  }

  bool is_armed() const
  {
    return have_vehicle_status_ &&
           vehicle_status_.arming_state ==
           px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool battery_ok() const
  {
    if (!have_battery_) {
      return true;
    }

    const bool warning_ok =
      battery_.warning == px4_msgs::msg::BatteryStatus::WARNING_NONE ||
      battery_.warning == px4_msgs::msg::BatteryStatus::WARNING_LOW;
    const bool remaining_ok =
      battery_.remaining < 0.0F ||
      battery_.remaining >= static_cast<float>(config_.battery_min_remaining);
    return warning_ok && remaining_ok;
  }

  aeris_safety::SafetyInputs build_inputs() const
  {
    const auto now = std::chrono::steady_clock::now();

    aeris_safety::SafetyInputs inputs;
    inputs.pose_valid = have_position_ && position_.xy_valid && position_.z_valid;
    inputs.pose_age_s = have_position_ ?
      age_s(last_position_time_, now) : 1.0e9;
    inputs.battery_ok = battery_ok();
    inputs.battery_remaining = have_battery_ ?
      static_cast<double>(battery_.remaining) : -1.0;

    if (have_position_ && position_.xy_valid) {
      inputs.distance_from_home_m = std::hypot(
        static_cast<double>(position_.x) - geofence_center_x_,
        static_cast<double>(position_.y) - geofence_center_y_);
      inputs.geofence_ok =
        inputs.distance_from_home_m <= config_.geofence_radius_m;
    }

    inputs.mission_active = mission_active_;
    inputs.mission_heartbeat_age_s = have_mission_status_ ?
      age_s(last_mission_status_time_, now) : 1.0e9;

    inputs.force_stale_pose = force_stale_pose_;
    inputs.force_low_battery = force_low_battery_;
    inputs.force_geofence_breach = force_geofence_breach_;
    inputs.force_mission_heartbeat_loss = force_mission_heartbeat_loss_;
    return inputs;
  }

  bool telemetry_ready() const
  {
    return have_position_ && have_vehicle_status_;
  }

  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();
    const bool armed = is_armed();

    // Do not declare a fault until the supervisor has received the core PX4
    // telemetry. This avoids a false stale-pose alarm during DDS discovery.
    aeris_safety::SafetyEvaluation evaluation;
    if (telemetry_ready()) {
      evaluation = aeris_safety::evaluate_safety(build_inputs(), config_);
    }

    if (!evaluation.safe && armed && evaluation.action != "none") {
      const bool needs_dispatch =
        !action_sent_ || evaluation.action != last_dispatched_action_;
      if (needs_dispatch &&
        age_s(last_action_attempt_time_, now) >= action_retry_interval_s_)
      {
        last_action_attempt_time_ = now;
        if (dispatch_action(evaluation.action)) {
          action_sent_ = true;
          last_dispatched_action_ = evaluation.action;
          RCLCPP_ERROR(
            get_logger(),
            "SAFETY ACTION '%s' for fault '%s' (%zu active fault(s)).",
            evaluation.action.c_str(), evaluation.trigger.c_str(),
            evaluation.active_faults.size());
        }
      }
    } else if (evaluation.safe || !armed) {
      action_sent_ = false;
      last_dispatched_action_.clear();
    }

    publish_status(evaluation);
  }

  bool dispatch_action(const std::string & action)
  {
    if (action == "land") {
      return call_trigger(land_client_, action);
    }
    if (action == "rtl") {
      return call_trigger(rtl_client_, action);
    }
    if (action == "hold") {
      return call_trigger(hold_client_, action);
    }
    return false;
  }

  bool call_trigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & action)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Safety action '%s' waiting for service '%s'.",
        action.c_str(), client->get_service_name());
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
      request,
      [this, action](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(
            get_logger(), "Safety action '%s' accepted: %s",
            action.c_str(), response->message.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "Safety action '%s' rejected: %s",
            action.c_str(), response->message.c_str());
          action_sent_ = false;
        }
      });
    return true;
  }

  void publish_status(const aeris_safety::SafetyEvaluation & evaluation)
  {
    const auto now = std::chrono::steady_clock::now();

    aeris_interfaces::msg::SafetyStatus msg;
    msg.stamp = this->now();
    msg.safe = evaluation.safe;
    msg.armed = is_armed();
    msg.offboard = have_control_mode_ &&
      control_mode_.flag_control_offboard_enabled;
    msg.action = evaluation.action;
    msg.trigger = evaluation.trigger;

    msg.pose_valid = have_position_ && position_.xy_valid && position_.z_valid;
    msg.pose_age_s = have_position_ ?
      age_s(last_position_time_, now) : -1.0;

    msg.battery_ok = battery_ok();
    msg.battery_remaining = have_battery_ ?
      static_cast<double>(battery_.remaining) : -1.0;

    if (have_position_ && position_.xy_valid) {
      msg.distance_from_home_m = std::hypot(
        static_cast<double>(position_.x) - geofence_center_x_,
        static_cast<double>(position_.y) - geofence_center_y_);
      msg.geofence_ok =
        msg.distance_from_home_m <= config_.geofence_radius_m;
    } else {
      msg.distance_from_home_m = 0.0;
      msg.geofence_ok = true;
    }

    msg.mission_heartbeat_ok = !mission_active_ || (
      have_mission_status_ &&
      age_s(last_mission_status_time_, now) <=
      config_.mission_heartbeat_timeout_s);
    msg.mission_heartbeat_age_s = have_mission_status_ ?
      age_s(last_mission_status_time_, now) : -1.0;

    msg.active_faults = evaluation.active_faults;
    status_pub_->publish(msg);
  }

  void on_inject_fault(
    const std::shared_ptr<aeris_interfaces::srv::InjectFault::Request> request,
    std::shared_ptr<aeris_interfaces::srv::InjectFault::Response> response)
  {
    const std::string fault = request->fault;
    const bool enable = request->enable;

    auto set_fault = [this, enable](const std::string & name) -> bool
      {
        if (name == "stale_pose") {
          force_stale_pose_ = enable;
        } else if (name == "low_battery") {
          force_low_battery_ = enable;
        } else if (name == "geofence_breach") {
          force_geofence_breach_ = enable;
        } else if (name == "mission_heartbeat_loss") {
          force_mission_heartbeat_loss_ = enable;
        } else {
          return false;
        }
        return true;
      };

    bool known = false;
    if (fault == "all") {
      known = true;
      force_stale_pose_ = enable;
      force_low_battery_ = enable;
      force_geofence_breach_ = enable;
      force_mission_heartbeat_loss_ = enable;
    } else {
      known = set_fault(fault);
    }

    if (!known) {
      response->success = false;
      response->message = "unknown fault: " + fault;
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Fault injection: fault='%s' enable=%s",
      fault.c_str(), enable ? "true" : "false");
    response->success = true;
    response->message =
      std::string(enable ? "enabled " : "cleared ") + fault;
  }

  void on_clear_faults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    force_stale_pose_ = false;
    force_low_battery_ = false;
    force_geofence_breach_ = false;
    force_mission_heartbeat_loss_ = false;
    response->success = true;
    response->message = "all injected faults cleared";
    RCLCPP_INFO(get_logger(), "All injected faults cleared.");
  }

  // Parameters
  std::string pose_topic_;
  std::string battery_topic_;
  std::string vehicle_status_topic_;
  std::string control_mode_topic_;
  std::string mission_status_topic_;
  std::string status_topic_;
  std::string land_service_name_;
  std::string rtl_service_name_;
  std::string hold_service_name_;
  double publish_rate_hz_{10.0};
  double action_retry_interval_s_{2.0};
  double geofence_center_x_{0.0};
  double geofence_center_y_{0.0};
  aeris_safety::SafetyConfig config_;

  // Telemetry
  px4_msgs::msg::VehicleLocalPosition position_;
  px4_msgs::msg::BatteryStatus battery_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleControlMode control_mode_;

  bool have_position_{false};
  bool have_battery_{false};
  bool have_vehicle_status_{false};
  bool have_control_mode_{false};
  bool have_mission_status_{false};
  bool mission_active_{false};

  std::chrono::steady_clock::time_point last_position_time_;
  std::chrono::steady_clock::time_point last_mission_status_time_;

  // Injected faults
  bool force_stale_pose_{false};
  bool force_low_battery_{false};
  bool force_geofence_breach_{false};
  bool force_mission_heartbeat_loss_{false};

  // Action dispatch
  bool action_sent_{false};
  std::string last_dispatched_action_;
  std::chrono::steady_clock::time_point last_action_attempt_time_;

  // ROS interfaces
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    position_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr
    control_mode_sub_;
  rclcpp::Subscription<aeris_interfaces::msg::MissionStatus>::SharedPtr
    mission_status_sub_;
  rclcpp::Publisher<aeris_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Service<aeris_interfaces::srv::InjectFault>::SharedPtr inject_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rtl_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hold_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
