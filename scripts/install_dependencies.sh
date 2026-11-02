#!/usr/bin/env bash
# Install the system toolchain and ROS 2 Jazzy on Ubuntu/Pop!_OS 24.04.
# Run as your normal user; the script calls sudo when needed.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: run as your normal user, not with sudo." >&2
  echo "The script requests sudo for the commands that need it." >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

. /etc/os-release
if [[ "${VERSION_ID:-}" != "24.04" && "${AERIS_FORCE:-0}" != "1" ]]; then
  echo "ERROR: expected Ubuntu/Pop!_OS 24.04, found ${PRETTY_NAME:-unknown}." >&2
  echo "Set AERIS_FORCE=1 to continue anyway." >&2
  exit 1
fi

echo "[1/5] Checking network and disk"
curl -fsI --max-time 20 http://packages.ros.org/ros2/ubuntu/dists/noble/Release >/dev/null
curl -fsI --max-time 20 https://github.com >/dev/null
AVAIL_GB="$(df --output=avail -BG / | tail -1 | tr -dc '0-9')"
if [[ -n "$AVAIL_GB" ]] && (( AVAIL_GB < 25 )); then
  echo "WARNING: only ${AVAIL_GB} GiB free on /; PX4 + Gazebo may need about 20 GiB." >&2
fi

sudo -v

echo "[2/5] Installing base build tools"
sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  wget \
  gnupg \
  lsb-release \
  software-properties-common \
  git \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  ccache \
  python3 \
  python3-dev \
  python3-pip \
  python3-venv \
  python3-setuptools \
  python3-wheel \
  python3-pytest \
  libssl-dev \
  libasio-dev \
  libtinyxml2-dev \
  terminator

echo "[3/5] Configuring the official ROS 2 apt source"
ROS_APT_SOURCE_VERSION="1.3.0"
ROS_CODENAME="${VERSION_CODENAME:-noble}"
ROS_DEB="ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ROS_CODENAME}_all.deb"
if [[ -e /etc/apt/sources.list.d/ros2.sources ]]; then
  echo "ROS 2 apt source already present; leaving it unchanged."
else
  TMP_DEB="$(mktemp --suffix=.deb)"
  trap 'rm -f "$TMP_DEB"' EXIT
  curl -fL --retry 3 -o "$TMP_DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/${ROS_DEB}"
  sudo dpkg -i "$TMP_DEB"
  rm -f "$TMP_DEB"
  trap - EXIT
fi

echo "[4/5] Installing ROS 2 Jazzy"
sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep

echo "[5/5] Initializing rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
else
  echo "rosdep sources already initialized."
fi
if ! rosdep update; then
  echo "WARNING: rosdep update failed. Re-run 'rosdep update' later." >&2
fi

echo
echo "ROS 2 Jazzy installation complete."
echo "Verify:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 --version"
echo "  printenv ROS_DISTRO"
echo
echo "Next: bash $ROOT/scripts/bootstrap_px4.sh"
