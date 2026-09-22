#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
cd "$DRONE_ROOT/vendor/PX4-Autopilot"
for name in px4-gcc13-cstdint px4-inspection-dds-topics; do
  patch_file="$DRONE_ROOT/patches/$name.patch"
  if git apply --check "$patch_file" 2>/dev/null; then
    git apply "$patch_file"
  else
    git apply --reverse --check "$patch_file"
  fi
done
# GCC 13 warns about the old generic Matrix<M=1> inversion in the optional LPE.
# Retain the warning, but do not treat this one diagnostic class as an error.
# The simulation uses the stock EKF2; no estimator algorithm is patched.
DONT_RUN=1 make -j6 px4_sitl_default CMAKE_ARGS=-DCMAKE_CXX_FLAGS=-Wno-error=array-bounds
