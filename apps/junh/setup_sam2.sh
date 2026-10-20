#!/bin/bash
# Compatibility entry point for shared, pinned SAM2 provisioning.
set -euo pipefail
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$APP_DIR/../../scripts/setup_discovery_sam2.sh" \
  --runtime-dir "$APP_DIR/.sam2-runtime" --venv "$APP_DIR/.venv-sam2" \
  --checkpoint "$APP_DIR/.models/sam2.1_hiera_tiny.pt" \
  --config "$APP_DIR/config/mujoco.json" "$@"
