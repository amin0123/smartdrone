#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
agent_source="$DRONE_ROOT/vendor/Micro-XRCE-DDS-Agent"
for name in xrce-agent-fastdds-pin xrce-agent-gcc13; do
  patch_file="$DRONE_ROOT/patches/$name.patch"
  if git -C "$agent_source" apply --check "$patch_file" 2>/dev/null; then
    git -C "$agent_source" apply "$patch_file"
  else
    git -C "$agent_source" apply --reverse --check "$patch_file"
  fi
done
cmake -S "$DRONE_ROOT/vendor/Micro-XRCE-DDS-Agent" -B "$DRONE_ROOT/.local/build-xrce" \
  -DCMAKE_INSTALL_PREFIX="$DRONE_ROOT/.local/xrce" \
  -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_BUILD_EXAMPLES=OFF
cmake --build "$DRONE_ROOT/.local/build-xrce" -j6
cmake --install "$DRONE_ROOT/.local/build-xrce"
