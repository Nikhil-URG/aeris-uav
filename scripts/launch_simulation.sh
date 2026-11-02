#!/usr/bin/env bash
# Start the Micro XRCE-DDS Agent and PX4 SITL with Gazebo.
#
#   bash scripts/launch_simulation.sh
#   HEADLESS=1 bash scripts/launch_simulation.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

PX4_DIR="${AERIS_PX4_DIR:-$HOME/aeris_dev/PX4-Autopilot}"
LOG_DIR="$ROOT/results/logs"
mkdir -p "$LOG_DIR"

if [[ ! -d "$PX4_DIR/.git" ]]; then
  echo "ERROR: PX4-Autopilot not found at $PX4_DIR." >&2
  echo "Run: bash scripts/bootstrap_px4.sh" >&2
  exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "ERROR: MicroXRCEAgent not found on PATH." >&2
  echo "Run: bash scripts/install_microxrce_agent.sh" >&2
  exit 1
fi

AGENT_PID=""
cleanup() {
  if [[ -n "$AGENT_PID" ]]; then
    echo "[launch] Stopping MicroXRCEAgent (PID $AGENT_PID)."
    kill "$AGENT_PID" 2>/dev/null || true
    wait "$AGENT_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if command -v ss >/dev/null 2>&1 && \
   ss -lun 2>/dev/null | awk '{print $5}' | grep -qE '[:.]8888$'; then
  echo "[launch] UDP 8888 is already in use; assuming an agent is running."
else
  echo "[launch] Starting MicroXRCEAgent on udp4:8888."
  MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/microxrce_agent.log" 2>&1 &
  AGENT_PID=$!
  sleep 1
  if ! kill -0 "$AGENT_PID" 2>/dev/null; then
    echo "[launch] Agent failed to start. Log follows:" >&2
    cat "$LOG_DIR/microxrce_agent.log" >&2 || true
    exit 1
  fi
fi

cd "$PX4_DIR"
echo "[launch] Starting PX4 SITL (headless=${HEADLESS:-0})."
echo "[launch] Agent log: $LOG_DIR/microxrce_agent.log"
echo "[launch] PX4 log:   $LOG_DIR/px4_sitl.log"
make px4_sitl gz_x500 2>&1 | tee "$LOG_DIR/px4_sitl.log"
