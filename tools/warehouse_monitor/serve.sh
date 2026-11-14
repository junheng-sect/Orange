#!/usr/bin/env bash
set -e
tool_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
source /home/orangepi/mid360s_ws/scripts/ros_net.sh
exec python3 "$tool_dir/monitor.py" "$@"
