#!/usr/bin/env bash
# Download QGroundControl as an AppImage into ~/Applications.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: run as your normal user, not with sudo." >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

QGC_VERSION="${QGC_VERSION:-v5.1.5}"
DEST="${QGC_DIR:-$HOME/Applications}"
APP="$DEST/QGroundControl.AppImage"
URL="https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/QGroundControl-x86_64.AppImage"

mkdir -p "$DEST"

if [[ ! -f "$APP" ]]; then
  echo "Downloading QGroundControl $QGC_VERSION ..."
  TMP_APP="$APP.part"
  trap 'rm -f "$TMP_APP"' EXIT
  curl -fL --retry 3 -o "$TMP_APP" "$URL"
  mv "$TMP_APP" "$APP"
  chmod +x "$APP"
  trap - EXIT
else
  echo "QGroundControl AppImage already present: $APP"
fi

if ! ldconfig -p 2>/dev/null | grep -q 'libfuse.so.2'; then
  echo "Installing AppImage and Qt xcb runtime libraries ..."
  sudo -v
  sudo apt-get update
  sudo apt-get install -y libfuse2t64 libxcb-cursor0 libxkbcommon-x11-0
else
  MISSING=()
  for pkg in libxcb-cursor0 libxkbcommon-x11-0; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
      MISSING+=("$pkg")
    fi
  done
  if (( ${#MISSING[@]} > 0 )); then
    sudo -v
    sudo apt-get install -y "${MISSING[@]}"
  fi
fi

sha256sum "$APP" | tee "$APP.sha256"

echo
echo "QGroundControl installed: $APP"
echo "Run: $ROOT/scripts/run_qgroundcontrol.sh"
