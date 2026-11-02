#!/usr/bin/env bash
# Build the AERIS ROS 2 workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

cd "$ROOT/ros2_ws"

if [[ ! -f src/px4_msgs/package.xml ]]; then
  echo "[build] Initializing the px4_msgs submodule ..."
  git -C "$ROOT" submodule update --init --recursive
fi

if [[ ! -f src/px4_msgs/package.xml ]]; then
  echo "ERROR: px4_msgs is missing at ros2_ws/src/px4_msgs." >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo
echo "Build complete."
echo "In a new shell:"
echo "  source $ROOT/scripts/aeris_env.sh"
