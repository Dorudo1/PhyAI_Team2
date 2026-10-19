#!/bin/bash
# Build and launch live MuJoCo RGB-D discovery from any working directory.
set -eo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$APP_DIR/../.." && pwd)"
SIM_WS="${JUNH_WS:-$(cd "$REPO_ROOT/../.." && pwd)}"
BUILD=true
SCAN_MODE="${AUTO_SEARCH:-true}"

case "${1:-}" in
    --no-build) BUILD=false; shift ;;
    -h|--help)
        cat <<'HELP'
Usage: apps/junh/run_sim.sh [--no-build]

Build and start MuJoCo, discovery, master, and workspace viewer.
--no-build reuses the existing workspace install.

Environment: ROS_DOMAIN_ID (42), HTTP_PORT (23517), JUNH_WS (repository's workspace).
Ctrl+C stops the launched processes. The arm swivels and captures wrist views automatically. Set AUTO_SEARCH=false for manual mode.
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
echo "MuJoCo search: http://localhost:$HTTP_PORT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, AUTO_SEARCH=$SCAN_MODE)"
echo "App workspace map: http://localhost:$HTTP_PORT/workspace"
exec ros2 launch junh_master mujoco.launch.py "http_port:=$HTTP_PORT" "auto_search:=$SCAN_MODE" "workspace_port:=$WORKSPACE_PORT"
