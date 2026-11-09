#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="/opt/ego-display/vendor:$ROS_PACKAGE_PATH"
exec "$@"
