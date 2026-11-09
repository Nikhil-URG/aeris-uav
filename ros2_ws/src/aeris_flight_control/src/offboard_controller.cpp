#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr uint8_t kPx4CustomMainModeAuto = 4;
constexpr uint8_t kPx4CustomMainModeOffboard = 6;
constexpr uint8_t kPx4CustomSubModeAutoLoiter = 3;
constexpr double kCommandResendPeriodS = 1.0;
constexpr double kCommandAckTimeoutS = 5.0;
}  // namespace

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : Node("aeris_offboard_controller")
  {
    // ---------------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------------
    auto_start_ = this->declare_parameter<bool>("auto_start", false);
    start_delay_s_ = this->declare_parameter<double>("start_delay_s", 5.0);
    takeoff_altitude_m_ = this->declare_parameter<double>("takeoff_altitude_m", 5.0);
    hold_duration_s_ = this->declare_parameter<double>("hold_duration_s", 10.0);
    setpoint_rate_hz_ = this->declare_parameter<double>("setpoint_rate_hz", 20.0);
    prestream_duration_s_ = this->declare_parameter<double>("prestream_duration_s", 2.0);
    mode_timeout_s_ = this->declare_parameter<double>("mode_timeout_s", 10.0);
    arm_timeout_s_ = this->declare_parameter<double>("arm_timeout_s", 10.0);
    takeoff_timeout_s_ = this->declare_parameter<double>("takeoff_timeout_s", 30.0);
    land_timeout_s_ = this->declare_parameter<double>("land_timeout_s", 45.0);
    position_tolerance_m_ = this->declare_parameter<double>("position_tolerance_m", 0.3);
    takeoff_speed_mps_ = this->declare_parameter<double>("takeoff_speed_mps", 1.0);
    max_altitude_m_ = this->declare_parameter<double>("max_altitude_m", 10.0);
    max_horizontal_distance_m_ =
      this->declare_parameter<double>("max_horizontal_distance_m", 20.0);
    max_hold_error_m_ = this->declare_parameter<double>("max_hold_error_m", 1.0);
    telemetry_timeout_s_ = this->declare_parameter<double>("telemetry_timeout_s", 1.0);
    abort_grace_s_ = this->declare_parameter<double>("abort_grace_s", 2.0);

    // Stay within safe bounds even if a bad YAML value is supplied.
    max_altitude_m_ = std::clamp(max_altitude_m_, 1.0, 50.0);
    takeoff_altitude_m_ = std::clamp(takeoff_altitude_m_, 0.5, max_altitude_m_);
    setpoint_rate_hz_ = std::clamp(setpoint_rate_hz_, 2.0, 50.0);
    takeoff_speed_mps_ = std::clamp(takeoff_speed_mps_, 0.1, 5.0);
    position_tolerance_m_ = std::clamp(position_tolerance_m_, 0.1, 2.0);
    max_horizontal_distance_m_ = std::clamp(max_horizontal_distance_m_, 1.0, 100.0);
    max_hold_error_m_ = std::clamp(max_hold_error_m_, 0.2, 10.0);
    telemetry_timeout_s_ = std::clamp(telemetry_timeout_s_, 0.2, 10.0);
    abort_grace_s_ = std::clamp(abort_grace_s_, 0.5, 10.0);
    start_delay_s_ = std::max(start_delay_s_, 0.0);
    hold_duration_s_ = std::max(hold_duration_s_, 0.0);

    // ---------------------------------------------------------------------
    // QoS: PX4 publishes best effort; the proven offboard example uses
    // reliable publishers for /fmu/in. Keep that split.
    // ---------------------------------------------------------------------
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", sub_qos,
      std::bind(&OffboardController::position_callback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sub_qos,
      std::bind(&OffboardController::status_callback, this, std::placeholders::_1));

    control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", sub_qos,
      std::bind(&OffboardController::control_mode_callback, this, std::placeholders::_1));

    command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", sub_qos,
      std::bind(&OffboardController::command_ack_callback, this, std::placeholders::_1));

    offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", pub_qos);

    trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", pub_qos);

    vehicle_command_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", pub_qos);

    start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/start",
      std::bind(&OffboardController::start_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    abort_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/abort",
      std::bind(&OffboardController::abort_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    node_start_time_ = std::chrono::steady_clock::now();
    state_enter_time_ = node_start_time_;
    last_ramp_time_ = node_start_time_;

    const auto setpoint_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / setpoint_rate_hz_));
    setpoint_timer_ = this->create_wall_timer(
      setpoint_period, std::bind(&OffboardController::setpoint_timer_callback, this));
    state_timer_ = this->create_wall_timer(
      100ms, std::bind(&OffboardController::state_timer_callback, this));
    heartbeat_timer_ = this->create_wall_timer(
      500ms, std::bind(&OffboardController::heartbeat_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "AERIS offboard controller initialized: auto_start=%s, takeoff=%.1f m, "
      "setpoint_rate=%.1f Hz, prestream=%.1f s, hold=%.1f s.",
      auto_start_ ? "true" : "false", takeoff_altitude_m_, setpoint_rate_hz_,
      prestream_duration_s_, hold_duration_s_);

    if (auto_start_) {
      RCLCPP_WARN(
        get_logger(),
        "auto_start is enabled: the vehicle will arm and take off after the "
        "start delay once preflight checks pass.");
    }

    RCLCPP_INFO(
      get_logger(),
      "Services: %s/start, %s/abort",
      get_name(), get_name());
  }

private:
  enum class State
  {
    WAIT_FOR_READY,
    PRESTREAM,
    REQUEST_OFFBOARD,
    ARM,
    TAKEOFF,
    HOLD,
    LAND,
    ABORT_LAND,
    DONE,
    ABORTED,
  };

  static const char * state_name(State state)
  {
    switch (state) {
      case State::WAIT_FOR_READY: return "WAIT_FOR_READY";
      case State::PRESTREAM: return "PRESTREAM";
      case State::REQUEST_OFFBOARD: return "REQUEST_OFFBOARD";
      case State::ARM: return "ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::HOLD: return "HOLD";
      case State::LAND: return "LAND";
      case State::ABORT_LAND: return "ABORT_LAND";
      case State::DONE: return "DONE";
      case State::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
  }

  static const char * command_name(uint32_t command)
  {
    switch (command) {
      case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE: return "DO_SET_MODE";
      case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
        return "ARM_DISARM";
      case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND: return "NAV_LAND";
      case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
        return "RETURN_TO_LAUNCH";
      default: return "COMMAND";
    }
  }

  // -----------------------------------------------------------------------
  // Callbacks
  // -----------------------------------------------------------------------
  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    position_ = *msg;
    have_position_ = true;
    last_position_time_ = std::chrono::steady_clock::now();

    if (msg->timestamp > 0) {
      px4_timestamp_us_ = msg->timestamp;
    }
  }

  void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    have_status_ = true;
    last_status_time_ = std::chrono::steady_clock::now();

    if (msg->timestamp > px4_timestamp_us_) {
      px4_timestamp_us_ = msg->timestamp;
    }
  }

  void control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    control_mode_ = *msg;
    have_control_mode_ = true;
    last_control_mode_time_ = std::chrono::steady_clock::now();
  }

  void command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    const bool accepted =
      msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

    if (accepted) {
      RCLCPP_INFO(
        get_logger(), "Command ack: %s accepted (result=%u).",
        command_name(msg->command), msg->result);
    } else {
      RCLCPP_WARN(
        get_logger(), "Command ack: %s result=%u param1=%u param2=%d.",
        command_name(msg->command), msg->result, msg->result_param1,
        msg->result_param2);
    }
  }

  void start_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ != State::WAIT_FOR_READY) {
      response->success = false;
      response->message = std::string("mission already active (state=") +
        state_name(state_) + ")";
      return;
    }

    if (!ready_to_start()) {
      response->success = false;
      response->message = "not ready: " + readiness_reason();
      return;
    }

    start_requested_ = true;
    response->success = true;
    response->message = "start accepted";
  }

  void abort_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    abort_requested_ = true;
    response->success = true;
    response->message = "abort requested";
  }

  // -----------------------------------------------------------------------
  // Timers
  // -----------------------------------------------------------------------
  void setpoint_timer_callback()
  {
    if (!streaming_ || !have_position_) {
      return;
    }

    publish_offboard_control_mode();
    publish_trajectory_setpoint();
  }

  void heartbeat_timer_callback()
  {
    // PX4's offboard check sets offboard_control_signal_lost=true when the
    // stream stops, which keeps pre_flight_checks_pass false even in POSCTL.
    // While idle, advertise that the offboard controller is alive so a later
    // mission can pass preflight checks again.
    if (streaming_ || !have_position_) {
      return;
    }

    publish_offboard_control_mode();
  }

  void state_timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();

    if (abort_requested_) {
      abort_requested_ = false;
      abort_mission("operator abort");
      return;
    }

    if (state_ != State::WAIT_FOR_READY &&
      state_ != State::DONE && state_ != State::ABORTED)
    {
      if (!have_position_ || age_s(last_position_time_, now) > telemetry_timeout_s_) {
        abort_mission("local position telemetry stale");
        return;
      }

      if (!have_status_ || age_s(last_status_time_, now) > telemetry_timeout_s_) {
        abort_mission("vehicle status telemetry stale");
        return;
      }

      if (!have_control_mode_ ||
        age_s(last_control_mode_time_, now) > telemetry_timeout_s_)
      {
        abort_mission("vehicle control mode telemetry stale");
        return;
      }

      if (status_.failsafe) {
        abort_mission("PX4 reports failsafe");
        return;
      }
    }

    switch (state_) {
      case State::WAIT_FOR_READY:
        handle_wait_for_ready(now);
        break;
      case State::PRESTREAM:
        handle_prestream(now);
        break;
      case State::REQUEST_OFFBOARD:
        handle_request_offboard(now);
        break;
      case State::ARM:
        handle_arm(now);
        break;
      case State::TAKEOFF:
        handle_takeoff(now);
        break;
      case State::HOLD:
        handle_hold(now);
        break;
      case State::LAND:
        handle_land(now);
        break;
      case State::ABORT_LAND:
        handle_abort_land(now);
        break;
      case State::DONE:
      case State::ABORTED:
        break;
    }
  }

  // -----------------------------------------------------------------------
  // State handlers
  // -----------------------------------------------------------------------
  void handle_wait_for_ready(const std::chrono::steady_clock::time_point & now)
  {
    if (!ready_to_start()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for takeoff readiness: %s", readiness_reason().c_str());
      return;
    }

    const bool auto_start_ready =
      auto_start_ && age_s(node_start_time_, now) >= start_delay_s_;

    if (!start_requested_ && !auto_start_ready) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 4000,
        "Ready for takeoff. Waiting for the start service%s.",
        auto_start_ ? " or auto-start delay" : "");
      return;
    }

    begin_mission();
  }

  void handle_prestream(const std::chrono::steady_clock::time_point & /*now*/)
  {
    if (elapsed_in_state_s() < prestream_duration_s_) {
      return;
    }

    transition_to(State::REQUEST_OFFBOARD, "offboard prestream complete");
    send_offboard_mode_command();
  }

  void handle_request_offboard(const std::chrono::steady_clock::time_point & now)
  {
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      abort_mission("vehicle armed before offboard request completed");
      return;
    }

    if (control_mode_.flag_control_offboard_enabled &&
      status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      transition_to(State::ARM, "offboard mode confirmed");
      send_arm_command();
      return;
    }

    if (age_s(last_command_time_, now) >= kCommandResendPeriodS) {
      send_offboard_mode_command();
    }

    if (elapsed_in_state_s() > mode_timeout_s_) {
      abort_mission("offboard mode request timed out");
    }
  }

  void handle_arm(const std::chrono::steady_clock::time_point & now)
  {
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      transition_to(State::TAKEOFF, "vehicle armed");
      target_z_ = position_.z;
      last_ramp_time_ = now;
      return;
    }

    if (age_s(last_command_time_, now) >= kCommandResendPeriodS) {
      send_arm_command();
    }

    if (elapsed_in_state_s() > arm_timeout_s_) {
      abort_mission("arming timed out");
    }
  }

  void handle_takeoff(const std::chrono::steady_clock::time_point & now)
  {
    if (status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ||
      status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      abort_mission("left armed offboard state during takeoff");
      return;
    }

    const double dt = std::max(0.0, age_s(last_ramp_time_, now));
    last_ramp_time_ = now;

    if (target_z_ > -takeoff_altitude_m_) {
      target_z_ = std::max(-takeoff_altitude_m_, target_z_ - takeoff_speed_mps_ * dt);
    }

    if (horizontal_distance_from_start() > max_horizontal_distance_m_) {
      abort_mission("horizontal safety limit exceeded during takeoff");
      return;
    }

    const double altitude_error = std::abs(position_.z - (-takeoff_altitude_m_));
    const bool target_reached = target_z_ <= -takeoff_altitude_m_ + 1e-4;
    const bool slow_enough = std::abs(position_.vz) < 0.2;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TAKEOFF: z=%.2f target_z=%.2f vz=%.2f altitude_error=%.2f",
      position_.z, target_z_, position_.vz, altitude_error);

    if (target_reached && altitude_error <= position_tolerance_m_ && slow_enough) {
      transition_to(State::HOLD, "takeoff altitude reached");
    }

    if (elapsed_in_state_s() > takeoff_timeout_s_) {
      abort_mission("takeoff timed out");
    }
  }

  void handle_hold(const std::chrono::steady_clock::time_point & /*now*/)
  {
    if (status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      abort_mission("vehicle disarmed during hold");
      return;
    }

    if (!control_mode_.flag_control_offboard_enabled) {
      // An operator or another component may command AUTO_LAND directly.
      // Treat that as the landing phase rather than a failure.
      if (status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        streaming_ = false;
        transition_to(State::LAND, "external AUTO_LAND detected");
        return;
      }

      abort_mission("lost offboard mode during hold");
      return;
    }

    const double error = position_error_from_target();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "HOLD: position=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) error=%.2f m",
      position_.x, position_.y, position_.z,
      target_x_, target_y_, target_z_, error);

    if (error > max_hold_error_m_) {
      if (!hold_error_active_) {
        hold_error_active_ = true;
        hold_error_time_ = std::chrono::steady_clock::now();
      } else if (age_s(hold_error_time_, std::chrono::steady_clock::now()) > 2.0) {
        abort_mission("position hold error exceeded limit");
        return;
      }
    } else {
      hold_error_active_ = false;
    }

    if (elapsed_in_state_s() >= hold_duration_s_) {
      transition_to(State::LAND, "hold complete");
      send_land_command();
    }
  }

  void handle_land(const std::chrono::steady_clock::time_point & now)
  {
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      send_auto_loiter_mode_command();
      streaming_ = false;
      transition_to(State::DONE, "landed and disarmed");
      log_summary();
      return;
    }

    const bool land_mode_active =
      status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND ||
      !control_mode_.flag_control_offboard_enabled;

    if (land_mode_active) {
      // PX4 accepted the landing command. Stop the offboard stream and let
      // the autopilot complete the descent.
      if (streaming_) {
        streaming_ = false;
        RCLCPP_INFO(get_logger(), "Landing mode active; offboard stream stopped.");
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "LAND: altitude=%.2f m, waiting for disarm.", position_.z);

    } else if (age_s(last_command_time_, now) >= kCommandResendPeriodS) {
      send_land_command();
    }

    if (elapsed_in_state_s() > land_timeout_s_) {
      RCLCPP_WARN(
        get_logger(),
        "Landing timed out after %.1f s; verify the vehicle in QGroundControl.",
        land_timeout_s_);
      streaming_ = false;
      transition_to(State::DONE, "land timeout");
      log_summary();
    }
  }

  void handle_abort_land(const std::chrono::steady_clock::time_point & now)
  {
    if (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      send_auto_loiter_mode_command();
      streaming_ = false;
      transition_to(State::ABORTED, "aborted and disarmed");
      return;
    }

    const bool land_mode_active =
      status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND ||
      !control_mode_.flag_control_offboard_enabled;

    if (land_mode_active || elapsed_in_state_s() > abort_grace_s_) {
      if (streaming_) {
        streaming_ = false;
        RCLCPP_INFO(get_logger(), "Abort landing active; offboard stream stopped.");
      }
      transition_to(State::ABORTED, "abort landing started");
      return;
    }

    if (age_s(last_command_time_, now) >= kCommandResendPeriodS) {
      send_land_command();
    }
  }

  // -----------------------------------------------------------------------
  // Mission helpers
  // -----------------------------------------------------------------------
  void begin_mission()
  {
    start_x_ = position_.x;
    start_y_ = position_.y;
    start_z_ = position_.z;
    start_yaw_ = std::isfinite(position_.heading) ? position_.heading : 0.0;

    target_x_ = start_x_;
    target_y_ = start_y_;
    target_z_ = start_z_;
    target_yaw_ = start_yaw_;
    target_initialized_ = true;

    streaming_ = true;
    hold_error_active_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Starting mission at NED (%.2f, %.2f, %.2f), yaw=%.2f rad; target altitude=%.1f m.",
      start_x_, start_y_, start_z_, start_yaw_, takeoff_altitude_m_);

    transition_to(State::PRESTREAM, "start accepted");
  }

  void abort_mission(const std::string & reason)
  {
    if (state_ == State::DONE || state_ == State::ABORTED) {
      return;
    }

    if (state_ == State::ABORT_LAND) {
      return;
    }

    RCLCPP_ERROR(get_logger(), "ABORT: %s", reason.c_str());

    const bool armed = have_status_ &&
      status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

    if (armed) {
      transition_to(State::ABORT_LAND, reason);
      send_land_command();
    } else {
      streaming_ = false;
      transition_to(State::ABORTED, reason);
    }
  }

  bool ready_to_start() const
  {
    const auto now = std::chrono::steady_clock::now();

    return have_position_ && position_valid() && have_status_ && have_control_mode_ &&
           age_s(last_position_time_, now) <= telemetry_timeout_s_ &&
           age_s(last_status_time_, now) <= telemetry_timeout_s_ &&
           age_s(last_control_mode_time_, now) <= telemetry_timeout_s_ &&
           status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
           status_.pre_flight_checks_pass && !status_.failsafe;
  }

  bool position_valid() const
  {
    return have_position_ && position_.xy_valid && position_.z_valid &&
           position_.v_xy_valid && position_.v_z_valid;
  }

  std::string readiness_reason() const
  {
    const auto now = std::chrono::steady_clock::now();

    if (!have_position_) {
      return "waiting for /fmu/out/vehicle_local_position_v1";
    }
    if (age_s(last_position_time_, now) > telemetry_timeout_s_) {
      return "local position telemetry stale";
    }
    if (!position_.xy_valid || !position_.z_valid) {
      return "local position estimate invalid";
    }
    if (!position_.v_xy_valid || !position_.v_z_valid) {
      return "local velocity estimate invalid";
    }
    if (!have_status_) {
      return "waiting for /fmu/out/vehicle_status_v1";
    }
    if (age_s(last_status_time_, now) > telemetry_timeout_s_) {
      return "vehicle status telemetry stale";
    }
    if (!have_control_mode_) {
      return "waiting for /fmu/out/vehicle_control_mode";
    }
    if (age_s(last_control_mode_time_, now) > telemetry_timeout_s_) {
      return "vehicle control mode telemetry stale";
    }
    if (status_.failsafe) {
      return "PX4 reports failsafe";
    }
    if (status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      return "vehicle is not disarmed";
    }
    if (!status_.pre_flight_checks_pass) {
      return "PX4 preflight checks are not passing";
    }
    return "ready";
  }

  double horizontal_distance_from_start() const
  {
    const double dx = position_.x - start_x_;
    const double dy = position_.y - start_y_;
    return std::hypot(dx, dy);
  }

  double position_error_from_target() const
  {
    const double dx = position_.x - target_x_;
    const double dy = position_.y - target_y_;
    const double dz = position_.z - target_z_;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void transition_to(State next, const std::string & reason)
  {
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s",
      state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_enter_time_ = std::chrono::steady_clock::now();
  }

  double elapsed_in_state_s() const
  {
    return age_s(state_enter_time_, std::chrono::steady_clock::now());
  }

  static double age_s(
    const std::chrono::steady_clock::time_point & then,
    const std::chrono::steady_clock::time_point & now)
  {
    return std::chrono::duration<double>(now - then).count();
  }

  // -----------------------------------------------------------------------
  // Publishing
  // -----------------------------------------------------------------------
  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = px4_timestamp_us_;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    if (!target_initialized_) {
      return;
    }

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = px4_timestamp_us_;
    msg.position = {
      static_cast<float>(target_x_),
      static_cast<float>(target_y_),
      static_cast<float>(target_z_)};
    msg.velocity = {0.0F, 0.0F, 0.0F};
    msg.acceleration = {0.0F, 0.0F, 0.0F};
    msg.jerk = {0.0F, 0.0F, 0.0F};
    msg.yaw = static_cast<float>(target_yaw_);
    msg.yawspeed = 0.0F;
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_vehicle_command(
    uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F,
    float param3 = 0.0F,
    float param4 = 0.0F,
    double param5 = 0.0,
    double param6 = 0.0,
    float param7 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = px4_timestamp_us_;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.confirmation = 0;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);

    last_command_time_ = std::chrono::steady_clock::now();
  }

  void send_offboard_mode_command()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F, static_cast<float>(kPx4CustomMainModeOffboard));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Requesting OFFBOARD mode.");
  }

  void send_arm_command()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Requesting arming.");
  }

  void send_land_command()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Requesting landing at current position.");
  }

  void send_auto_loiter_mode_command()
  {
    // AUTO_LOITER is a safe post-landing mode in this GCS-only SITL setup:
    // POSCTL requires a manual-control link, which is not present here.
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F, static_cast<float>(kPx4CustomMainModeAuto),
      static_cast<float>(kPx4CustomSubModeAutoLoiter));
    RCLCPP_INFO(get_logger(), "Restoring AUTO_LOITER mode after landing.");
  }

  void log_summary() const
  {
    RCLCPP_INFO(
      get_logger(),
      "Mission summary: state=%s position=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f).",
      state_name(state_), position_.x, position_.y, position_.z,
      target_x_, target_y_, target_z_);
  }

  // -----------------------------------------------------------------------
  // Parameters
  // -----------------------------------------------------------------------
  bool auto_start_{false};
  double start_delay_s_{5.0};
  double takeoff_altitude_m_{5.0};
  double hold_duration_s_{10.0};
  double setpoint_rate_hz_{20.0};
  double prestream_duration_s_{2.0};
  double mode_timeout_s_{10.0};
  double arm_timeout_s_{10.0};
  double takeoff_timeout_s_{30.0};
  double land_timeout_s_{45.0};
  double position_tolerance_m_{0.3};
  double takeoff_speed_mps_{1.0};
  double max_altitude_m_{10.0};
  double max_horizontal_distance_m_{20.0};
  double max_hold_error_m_{1.0};
  double telemetry_timeout_s_{1.0};
  double abort_grace_s_{2.0};

  // -----------------------------------------------------------------------
  // State and telemetry
  // -----------------------------------------------------------------------
  State state_{State::WAIT_FOR_READY};
  std::chrono::steady_clock::time_point node_start_time_;
  std::chrono::steady_clock::time_point state_enter_time_;
  std::chrono::steady_clock::time_point last_position_time_;
  std::chrono::steady_clock::time_point last_status_time_;
  std::chrono::steady_clock::time_point last_control_mode_time_;
  std::chrono::steady_clock::time_point last_ramp_time_;
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point hold_error_time_;

  bool have_position_{false};
  bool have_status_{false};
  bool have_control_mode_{false};
  bool start_requested_{false};
  bool abort_requested_{false};
  bool streaming_{false};
  bool target_initialized_{false};
  bool hold_error_active_{false};
  uint64_t px4_timestamp_us_{0};

  px4_msgs::msg::VehicleLocalPosition position_;
  px4_msgs::msg::VehicleStatus status_;
  px4_msgs::msg::VehicleControlMode control_mode_;

  double start_x_{0.0};
  double start_y_{0.0};
  double start_z_{0.0};
  double start_yaw_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_{0.0};

  // -----------------------------------------------------------------------
  // ROS interfaces
  // -----------------------------------------------------------------------
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_service_;

  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
