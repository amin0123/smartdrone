#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$DRONE_ROOT/scripts/activate.sh"
python -m pip install --only-binary=:all: --no-deps --target "$DRONE_ROOT/.local/isaac6-python" \
  pymavlink==2.4.49 lxml==5.3.0 fastcrc==0.3.2 PyYAML==6.0.2
source_dir="$DRONE_ROOT/vendor/PegasusSimulator6"
patch_file="$DRONE_ROOT/patches/pegasus6-physics-attitude.patch"
if git -C "$source_dir" apply --check "$patch_file"; then
  git -C "$source_dir" apply "$patch_file"
else
  git -C "$source_dir" apply --reverse --check "$patch_file"
fi
