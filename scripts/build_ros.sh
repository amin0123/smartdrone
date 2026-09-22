#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
cd "$DRONE_ROOT/ros2_ws"
python -m colcon build --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" "$@"
