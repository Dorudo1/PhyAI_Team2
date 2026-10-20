#!/bin/bash
# Provision SAM2, source its environment, then launch the shared ROS feature.
# Setup options precede --; ROS launch arguments follow it.
set -euo pipefail
DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISCOVERY_RUNTIME="${PHYAI_SAM2_RUNTIME_DIR:-$DISCOVERY_REPO/.discovery-sam2}"
DISCOVERY_SETUP_ARGS=()
while [ $# -gt 0 ] && [ "$1" != -- ]; do
  DISCOVERY_SETUP_ARGS+=("$1"); shift
done
if [ "${1:-}" = -- ]; then shift; fi
# The runtime location is selected by PHYAI_SAM2_RUNTIME_DIR, not a setup override.
for option in "${DISCOVERY_SETUP_ARGS[@]}"; do
  if [[ "$option" == --runtime-dir* ]]; then
    echo 'Set PHYAI_SAM2_RUNTIME_DIR to select the launcher cache.' >&2; exit 2
  fi
done
for option in "$@"; do
  if [[ "$option" == config_path:=* ]]; then
    echo 'Use --config /path/to/installation.json before -- so SAM2 is configured automatically.' >&2; exit 2
  fi
done
bash "$DISCOVERY_REPO/scripts/setup_discovery_sam2.sh" "${DISCOVERY_SETUP_ARGS[@]}"
source "$DISCOVERY_RUNTIME/env.sh"
exec ros2 launch phyai_discovery_ros discovery.launch.py \
  "config_path:=$PHYAI_DISCOVERY_CONFIG" "$@"
