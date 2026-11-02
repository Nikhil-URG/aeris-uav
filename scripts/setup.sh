#!/usr/bin/env bash
# One-command AERIS bootstrap.
#
# Run as your normal user. The individual scripts request sudo when needed.
# QGroundControl is optional and is not installed by this script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "============================================================"
echo " AERIS setup"
echo " Repository: $ROOT"
echo "============================================================"

echo
echo ">>> [1/4] System toolchain and ROS 2 Jazzy"
bash "$ROOT/scripts/install_dependencies.sh"

echo
echo ">>> [2/4] PX4-Autopilot v1.17.0 and Gazebo Harmonic"
bash "$ROOT/scripts/bootstrap_px4.sh"

echo
echo ">>> [3/4] Micro XRCE-DDS Agent v2.4.3"
bash "$ROOT/scripts/install_microxrce_agent.sh"

echo
echo ">>> [4/4] Building the ROS 2 workspace"
bash "$ROOT/scripts/build.sh"

echo
echo "============================================================"
echo " AERIS setup complete."
echo " Optional ground station:"
echo "   bash scripts/install_qgroundcontrol.sh"
echo " Run the simulation:"
echo "   bash scripts/launch_simulation.sh"
echo " In another terminal:"
echo "   source scripts/aeris_env.sh"
echo "   ros2 launch aeris_bringup telemetry.launch.py"
echo "============================================================"
