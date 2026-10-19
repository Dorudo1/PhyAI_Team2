#!/bin/bash
set -eo pipefail
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$APP_DIR/sam2_env.sh"
exec ros2 launch junh_master master.launch.py "$@"
