#!/usr/bin/env bash
set -euo pipefail
DRONE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="$DRONE_ROOT/.local/isaac-sim-5.1.0"
archive="$DRONE_ROOT/.local/downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip"
mkdir -p "$(dirname "$archive")"
if [[ -e "$target/VERSION" ]]; then
    [[ $(cat "$target/VERSION") == 5.1.0* ]] || { echo 'Unexpected Isaac version' >&2; exit 1; }
    echo "Isaac already extracted: $target"
    exit 0
fi
curl --fail --location --retry 3 --continue-at - --output "$archive" \
  https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
sha256sum "$archive" > "$archive.sha256"
unzip -tq "$archive"
mkdir -p "$target"
unzip -q -n "$archive" -d "$target"
(cd "$target" && ./post_install.sh)
echo "Installed private simulator: $target"
