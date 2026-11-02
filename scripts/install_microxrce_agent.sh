#!/usr/bin/env bash
# Build and install the Micro XRCE-DDS Agent pinned by AERIS.
# Run as your normal user; sudo is used only for the install step.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: run as your normal user, not with sudo." >&2
  exit 1
fi

VERSION="${MICRO_XRCE_AGENT_VERSION:-v2.4.3}"
SRC_DIR="${AERIS_XRCE_DIR:-$HOME/aeris_dev/Micro-XRCE-DDS-Agent}"

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "MicroXRCEAgent is already installed at $(command -v MicroXRCEAgent)."
  echo "Remove /usr/local/bin/MicroXRCEAgent and rerun to rebuild."
  exit 0
fi

for cmd in git cmake make g++; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command '$cmd' is missing." >&2
    echo "Run: bash scripts/install_dependencies.sh" >&2
    exit 1
  fi
done

# Keep conda out of the C++ build. A mis-selected Python can confuse CMake.
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

mkdir -p "$(dirname "$SRC_DIR")"
if [[ ! -d "$SRC_DIR/.git" ]]; then
  echo "Cloning Micro-XRCE-DDS-Agent $VERSION ..."
  git clone --branch "$VERSION" --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC_DIR"
else
  echo "Using existing checkout at $SRC_DIR; pinning $VERSION ..."
  git -C "$SRC_DIR" fetch --tags --force
  git -C "$SRC_DIR" checkout "$VERSION"
fi

echo "Configuring ..."
cmake -S "$SRC_DIR" -B "$SRC_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAGENT_BUILD_EXECUTABLE=ON

echo "Building (this can take several minutes) ..."
cmake --build "$SRC_DIR/build" --parallel "$(nproc)"

sudo -v
sudo cmake --install "$SRC_DIR/build"
sudo ldconfig /usr/local/lib

echo
if command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "Micro XRCE-DDS Agent installed at $(command -v MicroXRCEAgent)."
else
  echo "ERROR: installation completed but MicroXRCEAgent is not on PATH." >&2
  exit 1
fi
echo "Next: bash scripts/build.sh"
