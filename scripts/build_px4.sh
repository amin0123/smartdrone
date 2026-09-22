#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/activate.sh"
cd "$DRONE_ROOT/vendor/PX4-Autopilot"
DONT_RUN=1 make -j6 px4_sitl_default
