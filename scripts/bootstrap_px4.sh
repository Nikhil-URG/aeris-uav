#!/usr/bin/env bash
# Clone and set up PX4-Autopilot v1.17.0 with Gazebo Harmonic.
# Run as your normal user; PX4's setup script calls sudo when needed.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: run as your normal user, not with sudo." >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Keep the Miniforge Python 3.13 out of PX4's setup and build.
_aeris_strip_conda() {
  local value="$1"
  printf '%s' "$value" \
    | tr ':' '\n' \
    | grep -vE '/(miniforge3|miniconda3|anaconda3|conda)/' \
    | paste -sd: - || true
}
PATH="$(_aeris_strip_conda "${PATH:-}")"
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

if ! python3 -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 12) else 1)' 2>/dev/null; then
  echo "ERROR: PX4 setup requires system Python 3.12." >&2
  echo "python3 currently resolves to: $(command -v python3 || echo missing)" >&2
  python3 --version 2>&1 || true
  echo "Fix PATH or run 'source scripts/aeris_env.sh' first." >&2
  exit 1
fi

if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "ERROR: python3 resolves to $(command -v python3), expected /usr/bin/python3." >&2
  exit 1
fi

for cmd in git cmake ninja lsb_release; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command '$cmd' is missing." >&2
    echo "Run: bash scripts/install_dependencies.sh" >&2
    exit 1
  fi
done

PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_DIR="${AERIS_PX4_DIR:-$HOME/aeris_dev/PX4-Autopilot}"

mkdir -p "$(dirname "$PX4_DIR")"
if [[ -d "$PX4_DIR" && ! -d "$PX4_DIR/.git" ]]; then
  echo "ERROR: $PX4_DIR exists but is not a Git repository." >&2
  exit 1
fi
if [[ ! -d "$PX4_DIR/.git" ]]; then
  echo "Cloning PX4-Autopilot into $PX4_DIR ..."
  git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

cd "$PX4_DIR"
echo "Fetching tags and checking out $PX4_VERSION ..."
git fetch --tags --force
git checkout "$PX4_VERSION"
git submodule update --init --recursive

echo "PX4 pinned at $(git describe --tags --exact-match 2>/dev/null || git rev-parse --short HEAD) ($(git rev-parse HEAD))"

sudo -v

SETUP_ARGS=()
if [[ "${AERIS_INSTALL_NUTTX:-0}" != "1" ]]; then
  SETUP_ARGS+=(--no-nuttx)
  echo "NuttX toolchain disabled (set AERIS_INSTALL_NUTTX=1 to install it)."
fi

echo "Running PX4 Tools/setup/ubuntu.sh ${SETUP_ARGS[*]:-} ..."
bash ./Tools/setup/ubuntu.sh "${SETUP_ARGS[@]}"

echo
echo "PX4 and Gazebo setup complete."
echo "Verify:"
echo "  cd $PX4_DIR && git describe --tags"
echo "  gz sim --version"
echo
echo "Next: bash $ROOT/scripts/install_microxrce_agent.sh"
