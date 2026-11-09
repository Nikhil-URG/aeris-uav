#!/usr/bin/env bash
# Spawn the tall obstacle used by the Week 5 lidar/replanning demo.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

MODEL="$ROOT/simulation/models/tall_obstacle/model.sdf"

if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: obstacle model not found: $MODEL" >&2
  exit 1
fi

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "ERROR: ros_gz_sim is not installed." >&2
  exit 1
fi

echo "[obstacle] Waiting for /world/default/create ..."
for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q "/world/default/create"; then
    break
  fi
  sleep 1
done

if ! gz service -l 2>/dev/null | grep -q "/world/default/create"; then
  echo "ERROR: Gazebo world service not available; is the simulation running?" >&2
  exit 1
fi

echo "[obstacle] Spawning tall_obstacle_wp02 at (4.0, 2.0, 3.0)"
ros2 run ros_gz_sim create \
  -world default \
  -file "$MODEL" \
  -name tall_obstacle_wp02 \
  -x 4.0 -y 2.0 -z 3.0

echo "[obstacle] Done."
