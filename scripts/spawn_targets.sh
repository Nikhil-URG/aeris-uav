#!/usr/bin/env bash
# Spawn the two red inspection targets used by the Week 4 perception demo.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

MODEL="$ROOT/simulation/models/red_target/model.sdf"

if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: target model not found: $MODEL" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not available." >&2
  exit 1
fi

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "ERROR: ros_gz_sim is not installed." >&2
  echo "Run: sudo apt install -y ros-jazzy-ros-gz" >&2
  exit 1
fi

echo "[targets] Waiting for /world/default/create ..."
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

spawn() {
  local name="$1" x="$2" y="$3"
  echo "[targets] Spawning $name at ($x, $y)"
  ros2 run ros_gz_sim create \
    -world default \
    -file "$MODEL" \
    -name "$name" \
    -x "$x" -y "$y" -z 0.05
}

spawn red_target_wp01 4.0 0.0
spawn red_target_wp02 4.0 4.0

echo "[targets] Done."
