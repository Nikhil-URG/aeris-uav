#!/usr/bin/env bash
# Read-only environment check for AERIS. Safe to run at any time.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PASS=0
WARN=0
FAIL=0

section() { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '[ OK ]   %s\n' "$1"; PASS=$((PASS + 1)); }
warn() { printf '[ WARN ] %s\n' "$1"; WARN=$((WARN + 1)); }
fail() { printf '[ FAIL ] %s\n' "$1"; FAIL=$((FAIL + 1)); }

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/scripts/aeris_env.sh" >/dev/null 2>&1 || true
fi

section "System"
. /etc/os-release
echo "OS:        ${PRETTY_NAME:-unknown}"
echo "Kernel:    $(uname -r)"
echo "CPU:       $(nproc) threads"
echo "RAM:       $(free -h | awk '/^Mem:/{print $2}')"
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "GPU:       $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -1)"
fi
echo "Disk free: $(df -h "$ROOT" | awk 'NR==2{print $4}')"
echo "Root:      $ROOT"

section "Python"
PYTHON_BIN="$(command -v python3 2>/dev/null || true)"
echo "python3:   ${PYTHON_BIN:-missing}"
if [[ -n "$PYTHON_BIN" ]]; then
  "$PYTHON_BIN" --version 2>&1 || true
  if [[ "$PYTHON_BIN" == "/usr/bin/python3" ]]; then
    ok "system Python 3.12 is selected"
  else
    warn "non-system Python active; run: source scripts/aeris_env.sh"
  fi
else
  fail "python3 not found"
fi

section "ROS 2"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ok "ROS 2 Jazzy installed"
else
  fail "ROS 2 Jazzy missing; run: bash scripts/install_dependencies.sh"
fi
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 on PATH ($(command -v ros2))"
  echo "ROS_DISTRO: ${ROS_DISTRO:-unset}"
else
  warn "ros2 not on PATH; source scripts/aeris_env.sh"
fi
if command -v colcon >/dev/null 2>&1; then
  ok "colcon installed ($(command -v colcon))"
else
  fail "colcon missing"
fi

section "Simulation"
if command -v gz >/dev/null 2>&1; then
  GZ_VERSION="$(gz sim --versions 2>/dev/null | head -1 | tr -d ' ')"
  if [[ -n "$GZ_VERSION" ]]; then
    ok "Gazebo installed ($GZ_VERSION)"
  else
    ok "Gazebo installed (version query failed)"
  fi
else
  fail "Gazebo missing; run: bash scripts/bootstrap_px4.sh"
fi

PX4_DIR="${AERIS_PX4_DIR:-$HOME/aeris_dev/PX4-Autopilot}"
if [[ -d "$PX4_DIR/.git" ]]; then
  ok "PX4 checkout present at $PX4_DIR"
  echo "PX4 tag:    $(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo unknown)"
  echo "PX4 commit: $(git -C "$PX4_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"
else
  fail "PX4 checkout missing; run: bash scripts/bootstrap_px4.sh"
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  ok "MicroXRCEAgent installed ($(command -v MicroXRCEAgent))"
else
  fail "MicroXRCEAgent missing; run: bash scripts/install_microxrce_agent.sh"
fi

section "AERIS workspace"
if [[ -f "$ROOT/ros2_ws/src/px4_msgs/package.xml" ]]; then
  ok "px4_msgs submodule initialized"
  echo "px4_msgs commit: $(git -C "$ROOT/ros2_ws/src/px4_msgs" rev-parse --short HEAD 2>/dev/null || echo unknown)"
else
  warn "px4_msgs submodule not initialized"
fi
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  ok "workspace is built"
else
  warn "workspace not built; run: bash scripts/build.sh"
fi
if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg list 2>/dev/null | grep -q '^aeris_flight_control$'; then
    ok "aeris_flight_control is discoverable"
  else
    warn "aeris_flight_control not discoverable; build/source the workspace"
  fi

  if ros2 pkg executables aeris_flight_control 2>/dev/null | grep -q 'offboard_controller'; then
    ok "offboard_controller executable available"
  else
    warn "offboard_controller executable missing; rebuild the workspace"
  fi

  if ros2 pkg list 2>/dev/null | grep -q '^aeris_interfaces$'; then
    ok "aeris_interfaces is discoverable"
  else
    warn "aeris_interfaces not discoverable; rebuild the workspace"
  fi

  if ros2 pkg executables aeris_mission_manager 2>/dev/null | grep -q 'mission_manager'; then
    ok "mission_manager executable available"
  else
    warn "mission_manager executable missing; rebuild the workspace"
  fi

  if ros2 pkg executables aeris_perception 2>/dev/null | grep -q 'perception'; then
    ok "perception executable available"
  else
    warn "perception executable missing; rebuild the workspace"
  fi

  if ros2 pkg executables aeris_navigation 2>/dev/null | grep -q 'navigation_node'; then
    ok "navigation_node executable available"
  else
    warn "navigation_node executable missing; rebuild the workspace"
  fi

  if ros2 pkg executables aeris_safety 2>/dev/null | grep -q 'safety_node'; then
    ok "safety_node executable available"
  else
    warn "safety_node executable missing; rebuild the workspace"
  fi

  if ros2 pkg list 2>/dev/null | grep -q '^ros_gz_bridge$'; then
    ok "ros_gz_bridge installed"
  else
    warn "ros_gz_bridge missing; camera bridge unavailable"
  fi
fi

section "Ground control"
QGC_APP="${QGC_APPIMAGE:-$HOME/Applications/QGroundControl.AppImage}"
if [[ -x "$QGC_APP" ]]; then
  ok "QGroundControl AppImage present"
else
  warn "QGroundControl not installed (optional)"
fi

printf '\nSummary: %d ok, %d warnings, %d failures\n' "$PASS" "$WARN" "$FAIL"
if (( FAIL > 0 )); then
  exit 1
fi
