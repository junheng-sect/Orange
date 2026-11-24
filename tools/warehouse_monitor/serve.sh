#!/usr/bin/env bash
set -e
tool_dir="$(cd "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")" && pwd)"
source /opt/ros/noetic/setup.bash
workspace="$(cd -- "$tool_dir/../.." && pwd -P)"
source "$workspace/devel/setup.bash"
source "$workspace/scripts/ros_net.sh"
exec python3 "$tool_dir/monitor.py" "$@"
