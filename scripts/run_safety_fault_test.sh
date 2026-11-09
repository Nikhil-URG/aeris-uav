#!/usr/bin/env bash
# Run one Week 6 safety fault scenario against a running PX4/Gazebo SITL.
#
# Prerequisites:
#   - PX4 SITL is running with a clean battery
#   - QGroundControl is connected (PX4 requires GCS for arming here)
#   - The AERIS workspace is built
#
# Usage: scripts/run_safety_fault_test.sh <stale_pose|low_battery|geofence_breach|mission_heartbeat_loss>
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

FAULT="${1:-}"
case "$FAULT" in
  stale_pose|low_battery|geofence_breach|mission_heartbeat_loss) ;;
  *)
    echo "Usage: $0 <stale_pose|low_battery|geofence_breach|mission_heartbeat_loss>" >&2
    exit 2
    ;;
esac

case "$FAULT" in
  stale_pose) EXPECTED_ACTION="land" ;;
  *) EXPECTED_ACTION="rtl" ;;
esac

LOG_DIR="$ROOT/results/logs"
MISSION_LOG="$LOG_DIR/week6_${FAULT}_mission.log"
SAFETY_LOG="$LOG_DIR/week6_${FAULT}_safety.log"
MISSION_FILE="$(ros2 pkg prefix aeris_bringup)/share/aeris_bringup/config/missions/safety_test.yaml"

MISSION_PID=""
SAFETY_PID=""

cleanup() {
  # Each launch runs in its own session/process group so that the whole
  # node tree is terminated, not just the ros2 launch front-end.
  for pid in "$MISSION_PID" "$SAFETY_PID"; do
    if [[ -n "$pid" ]]; then
      kill -TERM -"$pid" 2>/dev/null || true
    fi
  done
  sleep 3
  for pid in "$MISSION_PID" "$SAFETY_PID"; do
    if [[ -n "$pid" ]]; then
      kill -KILL -"$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

echo "[test] Starting mission stack for fault '$FAULT' (expected action: $EXPECTED_ACTION)."
setsid ros2 launch aeris_bringup mission.launch.py \
  mission_file:="$MISSION_FILE" >"$MISSION_LOG" 2>&1 &
MISSION_PID=$!

setsid ros2 launch aeris_bringup safety.launch.py >"$SAFETY_LOG" 2>&1 &
SAFETY_PID=$!

echo "[test] Waiting for offboard readiness (offboard heartbeat + GCS)."
READY=0
for _ in $(seq 1 90); do
  if grep -q "Ready for takeoff. Waiting for the start service" "$MISSION_LOG" 2>/dev/null; then
    READY=1
    break
  fi
  sleep 1
done
if [[ "$READY" -ne 1 ]]; then
  echo "[test] ERROR: offboard controller did not become ready." >&2
  tail -20 "$MISSION_LOG" >&2 || true
  exit 1
fi

echo "[test] Starting mission."
ros2 service call /aeris_mission_manager/start std_srvs/srv/Trigger >/dev/null

echo "[test] Waiting for the inspection hold."
INSPECT=0
for _ in $(seq 1 90); do
  if grep -q "\[INSPECT\]" "$MISSION_LOG" 2>/dev/null; then
    INSPECT=1
    break
  fi
  if grep -qE "\[ABORTED\]|Safety abort|Mission failed" "$MISSION_LOG" 2>/dev/null; then
    echo "[test] ERROR: mission ended before inspection." >&2
    tail -30 "$MISSION_LOG" >&2 || true
    exit 1
  fi
  sleep 1
done
if [[ "$INSPECT" -ne 1 ]]; then
  echo "[test] ERROR: mission did not reach the inspection hold." >&2
  exit 1
fi

echo "[test] Injecting fault: $FAULT"
ros2 service call /aeris_safety/inject_fault aeris_interfaces/srv/InjectFault \
  "{fault: $FAULT, enable: true}"

echo "[test] Waiting for the mission safety abort."
ABORTED=0
for _ in $(seq 1 40); do
  if grep -q "Safety abort" "$MISSION_LOG" 2>/dev/null; then
    ABORTED=1
    break
  fi
  sleep 1
done
if [[ "$ABORTED" -ne 1 ]]; then
  echo "[test] ERROR: mission did not record a safety abort." >&2
  tail -30 "$MISSION_LOG" >&2 || true
  exit 1
fi

echo "[test] Waiting for the safe action and disarm."
ACTION_OK=0
for _ in $(seq 1 120); do
  if ! grep -q "SAFETY ACTION '$EXPECTED_ACTION'" "$SAFETY_LOG" 2>/dev/null; then
    sleep 1
    continue
  fi
  ACTION_OK=1
  ARMED="$(ros2 topic echo --once /fmu/out/vehicle_status_v1 2>/dev/null \
    | awk '/^arming_state:/{print $2}' || true)"
  if [[ "$ARMED" == "1" ]]; then
    break
  fi
  sleep 2
done

if [[ "$ACTION_OK" -ne 1 ]]; then
  echo "[test] ERROR: safety action '$EXPECTED_ACTION' was not dispatched." >&2
  tail -20 "$SAFETY_LOG" >&2 || true
  exit 1
fi

FINAL_ARMED="$(ros2 topic echo --once /fmu/out/vehicle_status_v1 2>/dev/null \
  | awk '/^arming_state:/{print $2}' || true)"
if [[ "$FINAL_ARMED" != "1" ]]; then
  echo "[test] ERROR: vehicle did not disarm after the safe action." >&2
  exit 1
fi

echo "[test] PASS: fault '$FAULT' -> action '$EXPECTED_ACTION' -> disarmed."
echo "[test] Mission log: $MISSION_LOG"
echo "[test] Safety log:  $SAFETY_LOG"
grep -E "Safety abort|safety_abort" "$MISSION_LOG" | tail -3 || true
grep -E "SAFETY ACTION|Fault injection" "$SAFETY_LOG" | tail -5 || true
