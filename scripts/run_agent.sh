#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LD_LIBRARY_PATH="$DRONE_ROOT/.local/xrce/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$DRONE_ROOT/.local/xrce/bin/MicroXRCEAgent" udp4 -p 8888 "$@"
