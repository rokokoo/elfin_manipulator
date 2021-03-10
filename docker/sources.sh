#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/usr/share/gazebo/setup.sh"
source "/workspace/elfin_ws/devel/setup.sh"
exec "$@"