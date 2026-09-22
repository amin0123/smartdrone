#!/usr/bin/env python3
"""Fetch immutable dependency sources; never reset an existing checkout."""
from pathlib import Path
import subprocess
import yaml

ROOT = Path(__file__).resolve().parents[1]


def run(*args: str, cwd: Path | None = None) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def main() -> None:
    spec = yaml.safe_load((ROOT / 'config/dependencies.repos').read_text())
    for name, item in spec['repositories'].items():
        base = ROOT / ('ros2_ws/src' if name == 'px4_msgs' else 'vendor')
        base.mkdir(parents=True, exist_ok=True)
        path = base / name
        if not path.exists():
            run('git', 'init', str(path))
            run('git', 'remote', 'add', 'origin', item['url'], cwd=path)
            run('git', 'fetch', '--depth', '1', 'origin', item['version'], cwd=path)
            run('git', 'checkout', '--detach', 'FETCH_HEAD', cwd=path)
        actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=path, text=True).strip()
        if actual != item['version']:
            raise RuntimeError(f'{path}: expected {item["version"]}, found {actual}; refusing reset')
        if 'tag' in item:
            # PX4's CMake version parser needs the annotated release tag even
            # when the checkout itself is pinned by immutable commit.
            run('git', 'fetch', '--depth', '1', 'origin', 'tag', item['tag'], cwd=path)
        if name == 'PX4-Autopilot':
            run('git', 'submodule', 'update', '--init', '--recursive', '--depth', '1', cwd=path)
            # The SITL version-header generator still inspects NuttX tags.
            run('git', 'fetch', '--depth', '1', 'origin', 'tag', 'nuttx-10.3.0',
                cwd=path / 'platforms/nuttx/NuttX/nuttx')
        print(f'PIN VERIFIED {name}: {actual}', flush=True)


if __name__ == '__main__':
    main()
