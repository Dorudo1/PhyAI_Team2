#!/bin/bash
set -eo pipefail
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$APP_DIR/sam2_env.sh" ]; then
    source "$APP_DIR/sam2_env.sh"
elif [ -n "${PHYAI_SAM2_CHECKPOINT:-}" ]; then
    export JUNH_SAM2_CHECKPOINT="$PHYAI_SAM2_CHECKPOINT"
else
    echo 'SAM2 runtime missing; build with DISCOVERY_SAM2=true.' >&2
    exit 1
fi
exec ros2 launch junh_master master.launch.py "$@"
