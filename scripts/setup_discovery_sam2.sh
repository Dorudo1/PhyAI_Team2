#!/bin/bash
# Shared SAM2 provisioning. Remaining options are forwarded to the Python helper.
set -euo pipefail
DISCOVERY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$DISCOVERY_REPO/src/phyai_discovery/phyai_discovery/setup_sam2.py" \
  --runtime-dir "${PHYAI_SAM2_RUNTIME_DIR:-$DISCOVERY_REPO/.discovery-sam2}" \
  --config "$DISCOVERY_REPO/src/phyai_discovery_ros/config/discovery.json" "$@"
