#!/usr/bin/env bash
# AERIS environment bootstrap. Source this file; do not execute it.
#
#   source scripts/aeris_env.sh
#
# The development machine has Miniforge first in PATH, while ROS 2 Jazzy and
# PX4 SITL require the system Python 3.12 at /usr/bin/python3. This script
# removes conda from the current shell only. It does not modify ~/.bashrc.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: source this script instead of executing it:" >&2
  echo "  source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

AERIS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_aeris_strip_conda() {
  local value="$1"
  printf '%s' "$value" \
    | tr ':' '\n' \
    | grep -vE '/(miniforge3|miniconda3|anaconda3|conda)/' \
    | paste -sd: -
}

if [[ -n "${CONDA_PREFIX:-}" ]] && [[ "$(type -t conda 2>/dev/null || true)" == "function" ]]; then
  echo "[aeris_env] Deactivating conda environment '${CONDA_DEFAULT_ENV:-unknown}'."
  conda deactivate 2>/dev/null || true
fi

PATH="$(_aeris_strip_conda "${PATH:-}")"
if [[ -z "$PATH" ]]; then
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
fi
export PATH

if [[ -n "${PYTHONPATH:-}" ]]; then
  case "$PYTHONPATH" in
    *conda*|*miniforge*|*miniconda*|*anaconda*) unset PYTHONPATH ;;
  esac
fi

if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="$(_aeris_strip_conda "$LD_LIBRARY_PATH")"
  export LD_LIBRARY_PATH
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[aeris_env] ERROR: /opt/ros/jazzy/setup.bash not found." >&2
  echo "[aeris_env] Run: bash scripts/install_dependencies.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

if [[ -f "$AERIS_ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$AERIS_ROOT/ros2_ws/install/setup.bash"
fi

export AERIS_ROOT
export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-1}"

echo "[aeris_env] ROS_DISTRO=${ROS_DISTRO:-unset} | python3=$(command -v python3) | AERIS_ROOT=$AERIS_ROOT"
