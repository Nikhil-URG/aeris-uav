#include <string>

#include "aeris_safety/safety_evaluator.hpp"
#include "gtest/gtest.h"

namespace
{
aeris_safety::SafetyInputs nominal_inputs()
{
  aeris_safety::SafetyInputs inputs;
  inputs.pose_valid = true;
  inputs.pose_age_s = 0.02;
  inputs.battery_ok = true;
  inputs.battery_remaining = 0.95;
  inputs.geofence_ok = true;
  inputs.distance_from_home_m = 3.0;
  inputs.mission_active = true;
  inputs.mission_heartbeat_age_s = 0.1;
  return inputs;
}

bool has(
  const aeris_safety::SafetyEvaluation & evaluation,
  const std::string & fault)
{
  return aeris_safety::has_fault(evaluation.active_faults, fault.c_str());
}
}  // namespace

TEST(SafetyEvaluator, NominalFlightIsSafe)
{
  const auto evaluation = aeris_safety::evaluate_safety(
    nominal_inputs(), aeris_safety::SafetyConfig{});

  EXPECT_TRUE(evaluation.safe);
  EXPECT_TRUE(evaluation.active_faults.empty());
  EXPECT_EQ(evaluation.action, "none");
  EXPECT_EQ(evaluation.trigger, "none");
}

TEST(SafetyEvaluator, StalePoseCommandsLand)
{
  auto inputs = nominal_inputs();
  inputs.pose_age_s = 5.0;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "stale_pose"));
  EXPECT_EQ(evaluation.action, "land");
  EXPECT_EQ(evaluation.trigger, "stale_pose");
}

TEST(SafetyEvaluator, InvalidPoseCommandsLand)
{
  auto inputs = nominal_inputs();
  inputs.pose_valid = false;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "stale_pose"));
  EXPECT_EQ(evaluation.action, "land");
}

TEST(SafetyEvaluator, LowBatteryCommandsRtl)
{
  auto inputs = nominal_inputs();
  inputs.battery_remaining = 0.10;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "low_battery"));
  EXPECT_EQ(evaluation.action, "rtl");
  EXPECT_EQ(evaluation.trigger, "low_battery");
}

TEST(SafetyEvaluator, GeofenceBreachCommandsRtl)
{
  auto inputs = nominal_inputs();
  inputs.distance_from_home_m = 30.0;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "geofence_breach"));
  EXPECT_EQ(evaluation.action, "rtl");
  EXPECT_EQ(evaluation.trigger, "geofence_breach");
}

TEST(SafetyEvaluator, MissionHeartbeatLossCommandsRtl)
{
  auto inputs = nominal_inputs();
  inputs.mission_heartbeat_age_s = 10.0;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "mission_heartbeat_loss"));
  EXPECT_EQ(evaluation.action, "rtl");
  EXPECT_EQ(evaluation.trigger, "mission_heartbeat_loss");
}

TEST(SafetyEvaluator, StaleHeartbeatWithInactiveMissionIsSafe)
{
  auto inputs = nominal_inputs();
  inputs.mission_active = false;
  inputs.mission_heartbeat_age_s = 60.0;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_TRUE(evaluation.safe);
}

TEST(SafetyEvaluator, InjectedFaultsOverrideTelemetry)
{
  auto inputs = nominal_inputs();
  inputs.force_stale_pose = true;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "stale_pose"));
}

TEST(SafetyEvaluator, StalePoseTakesPriorityOverLowBattery)
{
  auto inputs = nominal_inputs();
  inputs.force_stale_pose = true;
  inputs.force_low_battery = true;

  const auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});

  EXPECT_FALSE(evaluation.safe);
  EXPECT_TRUE(has(evaluation, "stale_pose"));
  EXPECT_TRUE(has(evaluation, "low_battery"));
  EXPECT_EQ(evaluation.action, "land");
  EXPECT_EQ(evaluation.trigger, "stale_pose");
}

TEST(SafetyEvaluator, ClearingInjectedFaultsRestoresSafety)
{
  auto inputs = nominal_inputs();
  inputs.force_low_battery = true;

  auto evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});
  EXPECT_FALSE(evaluation.safe);

  inputs.force_low_battery = false;
  evaluation = aeris_safety::evaluate_safety(
    inputs, aeris_safety::SafetyConfig{});
  EXPECT_TRUE(evaluation.safe);
  EXPECT_EQ(evaluation.action, "none");
}

TEST(SafetyEvaluator, ActionsAreConfigurable)
{
  aeris_safety::SafetyConfig config;
  config.action_for_low_battery = "land";

  auto inputs = nominal_inputs();
  inputs.battery_remaining = 0.05;

  const auto evaluation = aeris_safety::evaluate_safety(inputs, config);
  EXPECT_EQ(evaluation.action, "land");
}
