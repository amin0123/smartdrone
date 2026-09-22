#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_path="${DRONE_ISAAC_PATH:-$DRONE_ROOT/.local/isaac-sim-5.1.0}"
[[ $(cat "$sim_path/VERSION") == 5.1.0* ]] || { echo 'Expected Isaac 5.1.0' >&2; exit 1; }
# ABI firewall: host Jazzy/conda Python libraries must not enter Kit's Python.
exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u VIRTUAL_ENV -u PYTHONHOME \
  PYTHONPATH="$DRONE_ROOT/.local/isaac-python:$DRONE_ROOT/vendor/PegasusSimulator/extensions/pegasus.simulator" \
  LD_LIBRARY_PATH="$sim_path/exts/isaacsim.ros2.bridge/jazzy/lib" \
  PATH="/usr/local/bin:/usr/bin:/bin" ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" PYTHONNOUSERSITE=1 ISAACSIM_PATH="$sim_path" \
  "$sim_path/python.sh" "$DRONE_ROOT/isaac_sim/scripts/run_simulation.py" "$@"
