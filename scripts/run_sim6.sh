#!/usr/bin/env bash
# Explicit experimental profile. See COMPATIBILITY.md before selecting it.
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_path="${DRONE_ISAAC6_PATH:-$HOME/isaacsim}"
expected_version='6.0.0-rc.59+release.41464.5f2772bc.gl'
[[ $(cat "$sim_path/VERSION") == "$expected_version" ]] || {
  echo "Expected qualified Isaac build $expected_version; see docs/COMPATIBILITY.md" >&2
  exit 1
}
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u VIRTUAL_ENV -u PYTHONHOME \
  PYTHONPATH="$DRONE_ROOT/.local/isaac6-python:$DRONE_ROOT/vendor/PegasusSimulator6/extensions/pegasus.simulator" \
  LD_LIBRARY_PATH="$sim_path/exts/isaacsim.ros2.core/jazzy/lib" \
  PATH="/usr/local/bin:/usr/bin:/bin" ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" PYTHONNOUSERSITE=1 DRONE_SIM_PROFILE=6 ISAACSIM_PATH="$sim_path" \
  "$sim_path/python.sh" "$DRONE_ROOT/isaac_sim/scripts/run_simulation.py" "$@"
