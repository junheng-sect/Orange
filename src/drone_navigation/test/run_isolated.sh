#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11431
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export PYTHONDONTWRITEBYTECODE=1
NAV_TEST_DIR=$(mktemp -d /tmp/drone-nav-check.XXXXXX)
export ROS_LOG_DIR="$NAV_TEST_DIR/roslogs"
NAV_TEST_PACKAGE=$(rospack find drone_navigation)
NAV_CORE_PID=''
NAV_LAUNCH_PID=''
cleanup() {
  if [ -n "$NAV_LAUNCH_PID" ]; then kill -INT -- "-$NAV_LAUNCH_PID" 2>/dev/null || true; fi
  if [ -n "$NAV_CORE_PID" ]; then kill -INT -- "-$NAV_CORE_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
# Do not reuse or disturb a master already present on this test port.
python3 - <<'PY'
import socket
s=socket.socket();s.settimeout(.2)
if s.connect_ex(('127.0.0.1',11431))==0:raise SystemExit('test port 11431 already in use')
PY
python3 - "$NAV_TEST_PACKAGE" "$NAV_TEST_DIR" <<'PY'
import sys,yaml
with open(sys.argv[1]+'/config/navigation.yaml') as f:c=yaml.safe_load(f)
c['config']['map_id']='synthetic';c['config']['lever_body']=[.1,0,0]
with open(sys.argv[2]+'/navigation.yaml','w') as f:yaml.safe_dump(c,f)
PY
setsid roscore -p 11431 > "$NAV_TEST_DIR/master.log" 2>&1 &
NAV_CORE_PID=$!
for _ in $(seq 1 30); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep .2
done
setsid roslaunch drone_navigation navigation.launch config:="$NAV_TEST_DIR/navigation.yaml" enable_output:=false > "$NAV_TEST_DIR/launch.log" 2>&1 &
NAV_LAUNCH_PID=$!
python3 -m unittest discover -s "$NAV_TEST_PACKAGE/test" -p 'test_*.py' -v > "$NAV_TEST_DIR/unit.log" 2>&1
set +e
timeout 100s python3 -u "$NAV_TEST_PACKAGE/test/integration.py" > "$NAV_TEST_DIR/integration.log" 2>&1
NAV_RESULT=$?
set -e
cat "$NAV_TEST_DIR/unit.log"
cat "$NAV_TEST_DIR/integration.log"
printf 'Evidence: %s\n' "$NAV_TEST_DIR"
if [ "$NAV_RESULT" -ne 0 ]; then tail -70 "$NAV_TEST_DIR/launch.log"; fi
exit "$NAV_RESULT"
