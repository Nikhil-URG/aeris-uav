#!/usr/bin/env bash
# Run the QGroundControl AppImage with the X11/XWayland Qt backend.
set -euo pipefail

APP="${QGC_APPIMAGE:-$HOME/Applications/QGroundControl.AppImage}"

if [[ ! -x "$APP" ]]; then
  echo "ERROR: QGroundControl AppImage not found or not executable: $APP" >&2
  echo "Run: bash scripts/install_qgroundcontrol.sh" >&2
  exit 1
fi

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec "$APP" "$@"
