#!/usr/bin/env bash
# Source this file; it affects only the current shell.
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! type conda >/dev/null 2>&1; then
    source "${DRONE_CONDA_ROOT:-$HOME/miniforge3}/etc/profile.d/conda.sh"
else
    source "$(conda info --base)/etc/profile.d/conda.sh"
fi
conda activate drone || return 1
source /opt/ros/jazzy/setup.bash
if [[ -f "$DRONE_ROOT/ros2_ws/install/local_setup.bash" ]]; then
    source "$DRONE_ROOT/ros2_ws/install/local_setup.bash"
fi
export DRONE_ROOT
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
