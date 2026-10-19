#!/bin/bash
# Build and launch live MuJoCo RGB-D discovery from any working directory.
set -eo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$APP_DIR/../.." && pwd)"
SIM_WS="${JUNH_WS:-$(cd "$REPO_ROOT/../.." && pwd)}"
BUILD=true
SCAN_MODE="${AUTO_SEARCH:-true}"
SCATTER_MODE="${SCATTER_BLOCKS:-true}"
SCATTER_SEED_VALUE="${SCATTER_SEED:--1}"
SCATTER_TABLE_VALUE="${SCATTER_TABLE:-all}"

case "${1:-}" in
    --no-build) BUILD=false; shift ;;
    -h|--help)
        cat <<'HELP'
Usage: apps/junh/run_sim.sh [--no-build]

Build and start MuJoCo, discovery, master, and workspace viewer.
--no-build reuses the existing workspace install.

Environment: ROS_DOMAIN_ID (42), HTTP_PORT (23517), JUNH_WS (repository's workspace),
SCATTER_BLOCKS (true), SCATTER_SEED (random), SCATTER_TABLE (all).
Ctrl+C stops the launched processes. The arm swivels and captures wrist views automatically. Set AUTO_SEARCH=false for manual mode.
Blocks scatter before the first camera frame or scan. Set SCATTER_BLOCKS=false to start with the aligned layout.
HELP
        exit 0 ;;
esac
if [ "$#" -ne 0 ]; then
    echo "Unknown argument: $1. Use --help for usage." >&2
    exit 1
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "Run this inside the ROS 2 Humble devcontainer." >&2
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export HTTP_PORT="${HTTP_PORT:-23517}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_ROOT/docker/fastdds_udp.xml"
if [[ ! "$HTTP_PORT" =~ ^[0-9]+$ ]] || (( 10#$HTTP_PORT < 1 || 10#$HTTP_PORT > 65535 )); then
    echo "HTTP_PORT must be an integer from 1 to 65535." >&2
    exit 1
fi

if [[ "$SCAN_MODE" != true && "$SCAN_MODE" != false ]]; then
    echo "AUTO_SEARCH must be true or false." >&2
    exit 1
fi
if [[ "$SCATTER_MODE" != true && "$SCATTER_MODE" != false ]]; then
    echo "SCATTER_BLOCKS must be true or false." >&2
    exit 1
fi
if [[ "$SCATTER_SEED_VALUE" != -1 && ! "$SCATTER_SEED_VALUE" =~ ^[0-9]+$ ]]; then
    echo "SCATTER_SEED must be a nonnegative integer." >&2
    exit 1
fi
case "$SCATTER_TABLE_VALUE" in
    all|0_0|0_1|1_0|1_1) ;;
    *) echo "SCATTER_TABLE must be all, 0_0, 0_1, 1_0, or 1_1." >&2; exit 1 ;;
esac
WORKSPACE_PORT="${WORKSPACE_PORT:-$((10#$HTTP_PORT+1))}"
if [[ ! "$WORKSPACE_PORT" =~ ^[0-9]+$ ]] || (( 10#$WORKSPACE_PORT < 1 || 10#$WORKSPACE_PORT > 65535 || 10#$WORKSPACE_PORT == 10#$HTTP_PORT )); then
    echo "WORKSPACE_PORT must be a different port from 1 to 65535." >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
mkdir -p "$SIM_WS"
cd "$SIM_WS"
if [ "$BUILD" = true ]; then
    colcon build --symlink-install \
        --base-paths "$APP_DIR" "$REPO_ROOT/src/phyai_discovery" "$REPO_ROOT/sim/piper_mujoco_sim" \
        --packages-select phyai_discovery junh_master piper_mujoco_sim
fi
if [ ! -f "$SIM_WS/install/setup.bash" ]; then
    echo "Workspace install missing. Run without --no-build first." >&2
    exit 1
fi
source "$SIM_WS/install/setup.bash"
source "$APP_DIR/sam2_env.sh"
echo "MuJoCo search: http://localhost:$HTTP_PORT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, AUTO_SEARCH=$SCAN_MODE, SCATTER_BLOCKS=$SCATTER_MODE, SCATTER_SEED=$SCATTER_SEED_VALUE, SCATTER_TABLE=$SCATTER_TABLE_VALUE)"
echo "App workspace map: http://localhost:$HTTP_PORT/workspace"
exec ros2 launch junh_master mujoco.launch.py "http_port:=$HTTP_PORT" "auto_search:=$SCAN_MODE" "workspace_port:=$WORKSPACE_PORT" \
    "scatter_on_start:=$SCATTER_MODE" "scatter_seed:=$SCATTER_SEED_VALUE" "scatter_table:=$SCATTER_TABLE_VALUE"
