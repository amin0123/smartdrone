#!/usr/bin/env python3
"""Compare the actual wire definitions consumed by our gateway, not comments."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
MESSAGES = (
    'VehicleStatus', 'VehicleLocalPosition', 'VehicleOdometry', 'VehicleAttitude',
    'BatteryStatus', 'VehicleLandDetected', 'OffboardControlMode',
    'TrajectorySetpoint', 'VehicleCommand', 'VehicleCommandAck',
)


def definition(path: Path) -> list[str]:
    lines = (re.sub(r'#.*', '', line).strip() for line in path.read_text().splitlines())
    # Constants are not serialized; whitespace has no semantic effect.
    return [' '.join(x.split()) for x in lines if x and '=' not in x]


def main() -> None:
    for name in MESSAGES:
        upstream = ROOT / f'vendor/PX4-Autopilot/msg/{name}.msg'
        ros = ROOT / f'ros2_ws/src/px4_msgs/msg/{name}.msg'
        assert definition(upstream) == definition(ros), f'{name}: PX4/ROS wire schema differs'
        print(f'{name}: matching wire fields')


if __name__ == '__main__':
    main()
