#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
cmake -S "$DRONE_ROOT/vendor/Micro-XRCE-DDS-Agent" -B "$DRONE_ROOT/.local/build-xrce" \
  -DCMAKE_INSTALL_PREFIX="$DRONE_ROOT/.local/xrce" \
  -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_BUILD_EXAMPLES=OFF
cmake --build "$DRONE_ROOT/.local/build-xrce" -j6
cmake --install "$DRONE_ROOT/.local/build-xrce"
