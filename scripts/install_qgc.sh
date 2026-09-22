#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
archive="$DRONE_ROOT/.local/downloads/QGroundControl-v5.0.8.AppImage"
destination="$DRONE_ROOT/.local/qgc-5.0.8"
mkdir -p "$(dirname "$archive")" "$destination"
curl -fL --retry 3 --continue-at - -o "$archive" \
  'https://github.com/mavlink/qgroundcontrol/releases/download/v5.0.8/QGroundControl-x86_64.AppImage'
sha256sum "$archive" > "$archive.sha256"
chmod u+x "$archive"
cd "$destination"
"$archive" --appimage-extract > "$DRONE_ROOT/logs/qgc-extract.log"
