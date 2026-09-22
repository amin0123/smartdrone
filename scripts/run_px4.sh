#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
px4_source="$DRONE_ROOT/vendor/PX4-Autopilot"
export PX4_SIM_MODEL=gazebo-classic_iris
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PATH="$DRONE_ROOT/px4/config:$PATH"
mkdir -p "$DRONE_ROOT/.local/px4-instance"
cd "$DRONE_ROOT/.local/px4-instance"
px4_build="$px4_source/build/px4_sitl_default"
# The compiled etc tree includes FTP metadata used by QGC; source ROMFS does not.
exec "$px4_build/bin/px4" "$px4_build/etc" -s "$px4_build/etc/init.d-posix/rcS" -i 0 "$@"
