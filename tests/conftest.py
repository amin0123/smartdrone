"""Pure algorithms are importable before the ROS interface build exists."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for package in (ROOT / 'ros2_ws/src').glob('drone_*'):
    sys.path.insert(0, str(package))
