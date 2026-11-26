#!/usr/bin/env bash
# Indoor3_1: private localhost master, synthetic z=0.5m, no resident services or FCU output.
set -eo pipefail
source /opt/ros/noetic/setup.bash
workspace="$(cd -- "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")/../../.." && pwd -P)"
source "${INDOOR_ORANGE_SETUP:-$workspace/devel/setup.bash}"
export ROS_MASTER_URI=http://127.0.0.1:11447 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
INDOOR_TEST_DIR="$(cd "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")" && pwd)"
INDOOR_EVIDENCE="${1:-$HOME/test_data/nav_indoor3_1_validation/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$INDOOR_EVIDENCE"
[ ! -e "$INDOOR_EVIDENCE/synthetic.bag" ] || { echo "evidence already exists"; exit 1; }
export ROS_LOG_DIR="$INDOOR_EVIDENCE/roslogs"
INDOOR_PIDS=()
cleanup() {
  local i p
  for ((i=${#INDOOR_PIDS[@]}-1;i>=0;i--)); do kill -INT -- "-${INDOOR_PIDS[i]}" 2>/dev/null || true; done
  for p in "${INDOOR_PIDS[@]}"; do wait "$p" || true; done
  printf 'Evidence: %s\n' "$INDOOR_EVIDENCE"
}
trap cleanup EXIT
python3 - <<'PY'
import socket
with socket.socket() as sock:
    sock.settimeout(.2)
    if sock.connect_ex(('127.0.0.1', 11447)) == 0:
        raise SystemExit('private master port 11447 already in use')
PY
python3 - "$INDOOR_TEST_DIR/../indoor_fly_nav.yaml" "$INDOOR_EVIDENCE/config.yaml" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
assert cfg['config']['allow_unknown_above'] is False
cfg['config']['map_id'] = 'synthetic'
with open(sys.argv[2], 'w') as output:
    yaml.safe_dump(cfg, output)
PY
setsid roscore -p 11447 >"$INDOOR_EVIDENCE/master.log" 2>&1 & INDOOR_PIDS+=("$!")
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam list >/dev/null
rosparam load "$INDOOR_EVIDENCE/config.yaml" /drone_nav
speed=.5
accel=.5
if [[ " $* " == *" --mode damper"* ]]; then
  source "$INDOOR_TEST_DIR/../common.sh"
  read -r speed accel _ _ <<< "$(nt_ego_limits "$INDOOR_EVIDENCE/config.yaml" test_indoor3_5)"
  [[ "$speed" == 0.4 && "$accel" == 0.4 ]]
fi
setsid roslaunch drone_navigation ego.launch radius:=0.4 half_height:=0.3 max_vel:="$speed" max_acc:="$accel" max_vel_z:=0.3 max_acc_z:=0.3 >"$INDOOR_EVIDENCE/ego.log" 2>&1 & INDOOR_PIDS+=("$!")
for node in waypoint_manager trajectory_executor offboard_gateway; do
  setsid rosrun drone_navigation "$node" _enable_output:=false >"$INDOOR_EVIDENCE/$node.log" 2>&1 & INDOOR_PIDS+=("$!")
done
if [[ " $* " == *" --mode damper_ground "* ]]; then
  printf '%s\n' '地面相机+模拟航线，无飞控输出' > "$INDOOR_EVIDENCE/validation_kind.txt"
  printf '%s\n' ov13855 > "$INDOOR_EVIDENCE/video_cameras.txt"
  printf '%s\n' 未标注 > "$INDOOR_EVIDENCE/damper_deg.txt"
  setsid roslaunch "$INDOOR_TEST_DIR/../indoor3_4_camera.launch" full:=true >"$INDOOR_EVIDENCE/camera.log" 2>&1 & INDOOR_PIDS+=("$!")
  setsid rosrun dual_camera_recorder dual_camera_recorder_node "_directory:=$INDOOR_EVIDENCE" _ov13855_only:=true _bag_prefix:=test_indoor3_5 >"$INDOOR_EVIDENCE/video.log" 2>&1 & INDOOR_PIDS+=("$!")
  setsid env RC_BAG_DIR="$INDOOR_EVIDENCE" RC_BAG_PREFIX=test_indoor3_5 "$workspace/scripts/rc_record/start_rc_record.sh" _rc_topic:=/fixture/ch6 >"$INDOOR_EVIDENCE/record.log" 2>&1 & INDOOR_PIDS+=("$!")
  timeout 25 rostopic echo -n1 --filter 'm.data' /indoor3_4_video/ready | grep -q True
fi
python3 -B -u "$INDOOR_TEST_DIR/integration_indoor3_1.py" --out "$INDOOR_EVIDENCE" "${@:2}" 2>&1 | tee "$INDOOR_EVIDENCE/integration.log"
