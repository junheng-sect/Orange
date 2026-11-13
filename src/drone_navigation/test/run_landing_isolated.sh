#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11448 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
LAND_TEST_DIR="${1:?evidence directory}"
mkdir -p "$LAND_TEST_DIR"
export ROS_LOG_DIR="$LAND_TEST_DIR/roslogs"
python3 - <<'PY'
import socket
with socket.socket() as s:assert s.connect_ex(('127.0.0.1',11448))!=0,'private test port occupied'
PY
LAND_TEST_PID=''
cleanup() { if [ -n "$LAND_TEST_PID" ]; then kill -INT -- "-$LAND_TEST_PID" 2>/dev/null || true; wait "$LAND_TEST_PID" || true; fi; }
trap cleanup EXIT
setsid roscore -p 11448 >"$LAND_TEST_DIR/master.log" 2>&1 & LAND_TEST_PID=$!
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam load /home/orangepi/mid360s_ws/scripts/nav_test/outdoor_nav_yaw45.yaml /drone_nav
python3 -B "$(dirname "$0")/integration_landing.py" "$LAND_TEST_DIR/results.json" >"$LAND_TEST_DIR/integration.log" 2>&1
cat "$LAND_TEST_DIR/integration.log"
