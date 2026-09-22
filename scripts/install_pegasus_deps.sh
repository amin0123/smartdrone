#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_path="$DRONE_ROOT/.local/isaac-sim-5.1.0"
# Import Pegasus through PYTHONPATH; its setup.py otherwise modifies Kit's base
# application. The standalone launcher explicitly enables needed extensions.
env -u PYTHONPATH -u LD_LIBRARY_PATH -u CONDA_PREFIX -u VIRTUAL_ENV \
  "$sim_path/python.sh" -m pip install --target "$DRONE_ROOT/.local/isaac-python" \
  --no-deps pymavlink==2.4.49 fastcrc==0.3.2 lxml==5.3.0 PyYAML==6.0.2
