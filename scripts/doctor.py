#!/usr/bin/env python3
"""Read-only dependency and ABI checks. A green result is not a flight test."""
import importlib
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['5', '6'], default='6')
    profile = parser.parse_args().profile
    checks: dict[str, dict] = {}

    def check(name: str, ok: bool, detail: str) -> None:
        checks[name] = {'ok': bool(ok), 'detail': detail}

    check('conda', os.environ.get('CONDA_DEFAULT_ENV') == 'drone', sys.executable)
    check('python', sys.version_info[:2] == (3, 12), sys.version)
    for module in ('rclpy', 'numpy', 'cv2', 'yaml', 'pytest', 'pymavlink'):
        try:
            imported = importlib.import_module(module)
            check(module, True, str(getattr(imported, '__version__', imported.__file__)))
        except Exception as exc:
            check(module, False, str(exc))
    isaac = (Path(os.environ.get('DRONE_ISAAC6_PATH', str(Path.home() / 'isaacsim')))
             if profile == '6' else ROOT / '.local/isaac-sim-5.1.0')
    pegasus = ROOT / 'vendor' / ('PegasusSimulator6' if profile == '6' else 'PegasusSimulator')
    for name, path in {
        'isaac': isaac / 'VERSION',
        'pegasus': pegasus / 'extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd',
        'px4': ROOT / 'vendor/PX4-Autopilot/build/px4_sitl_default/bin/px4',
        'xrce_agent': ROOT / '.local/xrce/bin/MicroXRCEAgent',
        'px4_msgs': ROOT / 'ros2_ws/install/px4_msgs/share/px4_msgs/package.xml',
        'qgc': ROOT / '.local/qgc-5.0.8/squashfs-root/AppRun',
    }.items():
        check(name, path.is_file(), str(path))
    result = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv,noheader'], capture_output=True, text=True) if shutil.which('nvidia-smi') else None
    check('gpu', result is not None and result.returncode == 0,
          result.stdout.strip() if result else 'nvidia-smi missing')
    print(json.dumps(checks, indent=2))
    return 0 if all(c['ok'] for c in checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
