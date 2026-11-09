#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace aeris_safety
{

struct SafetyConfig
{
  double pose_timeout_s{1.0};
  double battery_min_remaining{0.25};
  double geofence_radius_m{20.0};
  double mission_heartbeat_timeout_s{3.0};
  bool require_mission_heartbeat{true};

  std::string action_for_stale_pose{"land"};
  std::string action_for_low_battery{"rtl"};
  std::string action_for_geofence_breach{"rtl"};
  std::string action_for_mission_heartbeat_loss{"rtl"};
};

struct SafetyInputs
{
  bool pose_valid{false};
  double pose_age_s{0.0};
  bool battery_ok{true};
  double battery_remaining{1.0};
  bool geofence_ok{true};
  double distance_from_home_m{0.0};
  bool mission_active{false};
  double mission_heartbeat_age_s{0.0};

  bool force_stale_pose{false};
  bool force_low_battery{false};
  bool force_geofence_breach{false};
  bool force_mission_heartbeat_loss{false};
};

struct SafetyEvaluation
{
  bool safe{true};
  std::string action{"none"};
  std::string trigger{"none"};
  std::vector<std::string> active_faults;
};

inline bool has_fault(
  const std::vector<std::string> & faults,
  const char * name)
{
  return std::find(faults.begin(), faults.end(), name) != faults.end();
}

inline SafetyEvaluation evaluate_safety(
  const SafetyInputs & inputs,
  const SafetyConfig & config)
{
  SafetyEvaluation result;

  if (!inputs.pose_valid ||
    inputs.pose_age_s > config.pose_timeout_s ||
    inputs.force_stale_pose)
  {
    result.active_faults.emplace_back("stale_pose");
  }

  const bool remaining_known =
    inputs.battery_remaining >= 0.0 && inputs.battery_remaining <= 1.0;
  if (!inputs.battery_ok ||
    (remaining_known && inputs.battery_remaining < config.battery_min_remaining) ||
    inputs.force_low_battery)
  {
    result.active_faults.emplace_back("low_battery");
  }

  if (!inputs.geofence_ok ||
    inputs.distance_from_home_m > config.geofence_radius_m ||
    inputs.force_geofence_breach)
  {
    result.active_faults.emplace_back("geofence_breach");
  }

  if (inputs.force_mission_heartbeat_loss ||
    (inputs.mission_active && config.require_mission_heartbeat &&
    inputs.mission_heartbeat_age_s > config.mission_heartbeat_timeout_s))
  {
    result.active_faults.emplace_back("mission_heartbeat_loss");
  }

  result.safe = result.active_faults.empty();
  if (result.safe) {
    return result;
  }

  // Deterministic priority: loss of pose makes navigation unreliable, so
  // land immediately. A geofence breach is next, then battery, then loss of
  // the mission manager heartbeat.
  if (has_fault(result.active_faults, "stale_pose")) {
    result.trigger = "stale_pose";
    result.action = config.action_for_stale_pose;
  } else if (has_fault(result.active_faults, "geofence_breach")) {
    result.trigger = "geofence_breach";
    result.action = config.action_for_geofence_breach;
  } else if (has_fault(result.active_faults, "low_battery")) {
    result.trigger = "low_battery";
    result.action = config.action_for_low_battery;
  } else if (has_fault(result.active_faults, "mission_heartbeat_loss")) {
    result.trigger = "mission_heartbeat_loss";
    result.action = config.action_for_mission_heartbeat_loss;
  }

  return result;
}

}  // namespace aeris_safety
