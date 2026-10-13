#!/bin/bash
# Sources ROS and, once it has been built, the workspace, so every command starts ready to use.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f /apollo/ros2_ws/install/setup.bash ] && source /apollo/ros2_ws/install/setup.bash
exec "$@"
