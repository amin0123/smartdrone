#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
px4_source="$DRONE_ROOT/vendor/PX4-Autopilot"
export PX4_SIM_MODEL=gazebo-classic_iris
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PATH="$DRONE_ROOT/px4/config:$PATH"
mkdir -p "$DRONE_ROOT/.local/px4-instance"
cd "$DRONE_ROOT/.local/px4-instance"
exec "$px4_source/build/px4_sitl_default/bin/px4" "$px4_source/ROMFS/px4fmu_common" \
  -s "$px4_source/ROMFS/px4fmu_common/init.d-posix/rcS" -i 0 "$@"
