#!/bin/bash
# Source after the ROS workspace setup to enable local model dependencies.
JUNH_APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export JUNH_SAM2_CHECKPOINT="${JUNH_SAM2_CHECKPOINT:-$JUNH_APP_DIR/.models/sam2.1_hiera_tiny.pt}"
if [ ! -f "$JUNH_SAM2_CHECKPOINT" ] || [ ! -x "$JUNH_APP_DIR/.venv-sam2/bin/python" ]; then
    bash "$JUNH_APP_DIR/setup_sam2.sh" --checkpoint "$JUNH_SAM2_CHECKPOINT" || return 1
fi
SAM2_SITE="$("$JUNH_APP_DIR/.venv-sam2/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="$SAM2_SITE:${PYTHONPATH:-}"
