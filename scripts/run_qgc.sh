#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -u LD_LIBRARY_PATH -u QT_PLUGIN_PATH -u QML2_IMPORT_PATH \
  XDG_CONFIG_HOME="$DRONE_ROOT/.local/qgc-config" \
  XDG_DATA_HOME="$DRONE_ROOT/.local/qgc-data" \
  "$DRONE_ROOT/.local/qgc-5.0.8/squashfs-root/AppRun" "$@"
