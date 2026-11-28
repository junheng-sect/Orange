#!/usr/bin/env bash
# Real common.sh supervisor, private master and synthetic latched telemetry.
# shellcheck disable=SC2034,SC1091
source "$1/common.sh"
DIR=$2
MAP=$2
OUT=$2
NT_LOG_DIR=$2
RUN_DIR=$2/run
mkdir -p "$RUN_DIR"
LOG="$OUT/nav_test.log"
ROS_SETUP="$DIR/ros_net.sh" WS_SETUP="$DIR/ros_net.sh"
EV_MODE=odom LIVE_GRID_ONLY=1 NT_DLL=0 NT_RAW_LIDAR=0 PREVIEW=1
PREVIEW_RADIUS=.4 PREVIEW_HALF_HEIGHT=.3 MAP_FRAME='' TEST_NAME=fixture
RELOC_CONFIG="$MAP/reloc.yaml" NAV_CONFIG="$MAP/config.yaml"
PF_EV_CHECK="$RUN_DIR/ev.pid" PF_WAIT_ODOM="$RUN_DIR/wait.pid"
PF_FUSION="$RUN_DIR/fusion.pid" PF_RELOC="$RUN_DIR/reloc.pid" PF_NAV="$RUN_DIR/nav.pid"
PF_CAR="$RUN_DIR/car.pid"
PF_CAMERA="$RUN_DIR/camera.pid"
trap 'finish 143 force' TERM
test_desc() { echo fixture; }
test_bringup() { start_child "$PF_CAMERA" "$OUT/camera.log" camera; }
test_after_fusion() { child_pid "$PF_CAMERA" >/dev/null; }
test_cleanup() { stop_child "$PF_CAMERA" "相机" TERM 5; }
# Use the production pidfile launcher for every child. Substitute idle workers
# for the hardware and gateway and record whether the gateway was ever started.
eval "$(declare -f start_child | sed '1s/start_child/original_start_child/')"
start_child() {
  case $1 in
    "$PF_CAMERA"|"$PF_FUSION"|"$PF_RELOC"|"$PF_NAV")
      if [[ $1 == "$PF_FUSION" ]]; then child_pid "$PF_CAMERA" >/dev/null || exit 7; fi
      [[ $1 != "$PF_NAV" ]] || touch "$OUT/gateway.started"
      original_start_child "$1" "$2" python3 -c '
import os, signal, sys
def stop(*_): sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
with open(sys.argv[1], "w"):
    try: signal.pause()
    finally: os.unlink(sys.argv[1])
' "$1.alive" ;;
    *) original_start_child "$@" ;;
  esac
}
# Preserve identity files for inspection after production cleanup.
nt_archive_logs() { cp -a "$RUN_DIR/." "$OUT/identities/"; }
nt_bringup
finish 0 force
