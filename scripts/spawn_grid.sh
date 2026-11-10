#!/usr/bin/env bash
# Spawn the buildings defined by the AERIS city grid.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/aeris_env.sh"

exec python3 "$ROOT/scripts/spawn_buildings.py" "$@"
