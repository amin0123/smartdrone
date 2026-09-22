#!/usr/bin/env bash
# Observability only; never sends flight setpoints.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
cd "$DRONE_ROOT"
case "${1:-topics}" in
  qgc) exec bash scripts/run_qgc.sh ;;
  rviz) exec rviz2 -d "$DRONE_ROOT/config/inspection.rviz" ;;
  topics) exec ros2 topic list -t ;;
  state) exec ros2 topic echo /drone/state --qos-reliability best_effort ;;
  mission) exec ros2 topic echo /drone/mission_status --qos-durability transient_local ;;
  record)
    bag_path="logs/bags/inspection-$(date -u +%Y%m%dT%H%M%SZ)"
    mkdir -p logs/bags
    exec ros2 bag record -o "$bag_path" /clock /tf /tf_static /drone/state \
      /drone/odometry /drone/planned_path /drone/mission_status /drone/safety \
      /drone/battery /drone/detections /drone/dock_observation \
      /drone/camera/image_raw /drone/camera/camera_info /drone/camera/depth \
      /drone/command_result ;;
  *) echo 'Usage: bash scripts/debug.sh {qgc|rviz|topics|state|mission|record}' >&2; exit 2 ;;
esac
