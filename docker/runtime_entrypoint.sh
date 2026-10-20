#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash
if [ -f /opt/phyai-sam2/env.sh ]; then source /opt/phyai-sam2/env.sh; fi
exec "$@"
