#!/usr/bin/env bash
# Source the isolated build workspace before running.
set -eo pipefail

observe=0
if [[ "${1:-}" == --observe ]]; then
  observe=1
  shift
fi
if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: $0 [--observe] BAG MAP_DIR NEW_OUTPUT_DIR [SEED_JSON]" >&2
  exit 2
fi
bag=$(realpath -e "$1")
map=$(realpath -e "$2")
output=$(realpath -m "$3")
seed=${4:-}
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
port=${DLL_REPLAY_PORT:-11471}
if [[ ! "$port" =~ ^[0-9]{4,5}$ ]] || (( port < 1024 || port > 65535 || port == 11311 )); then
  echo "DLL_REPLAY_PORT must be a private replay port other than 11311" >&2
  exit 2
fi
export ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_LOG_DIR="$output/roslog"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

mkdir -p "$(dirname "$output")"
mkdir "$output"
if [[ -z "$seed" ]]; then
  mkdir "$output/seed_map"
  cp "$map/map_world.pcd" "$map/meta.json" "$output/seed_map/"
  if [[ -f "$map/reloc_cache_v2.npz" ]]; then
    cp "$map/reloc_cache_v2.npz" "$output/seed_map/"
  fi
  seed="$output/seed.json"
  rosrun drone_relocalization relocalize --map "$output/seed_map" --bag "$bag" \
    --at 0.2 --out "$seed" >"$output/seed.log" 2>&1
fi

seed_info=$(python3 - "$seed" "$map" "$bag" <<'PY'
import filecmp
import json
import math
from pathlib import Path
import sys

seed_path, map_dir, bag = map(Path, sys.argv[1:])
seed = json.loads(seed_path.read_text())
result = seed["results"][0]
if not result["ok"] or Path(seed["bag"]).resolve() != bag:
    raise SystemExit("The seed must be valid and computed from this bag")
seed_map = Path(seed["map"]) / "map_world.pcd"
if not filecmp.cmp(seed_map, map_dir / "map_world.pcd", shallow=False):
    # A benchmark may reuse a confirmed seed across crops of one unchanged
    # source map. Both crop manifests must identify the same original frame.
    def source_frame(folder):
        crop = json.loads((folder / "meta.json").read_text())["shared_outdoor_crop"]
        source = crop["source"]
        if crop["frame_id"] != "drone_map" or not source["map_sha256"] or not source["meta_sha256"]:
            raise ValueError("invalid source frame provenance")
        return source["map_sha256"], source["meta_sha256"]

    if seed.get("reuse_same_source_map_frame") is not True:
        raise SystemExit("The seed was computed against a different map")
    if source_frame(seed_map.parent) != source_frame(map_dir):
        raise SystemExit("The seed and target map do not share the original map frame")
transform = result["transform"]
values = transform["translation"] + transform["q_xyzw"]
if len(values) != 7 or not all(math.isfinite(v) for v in values):
    raise SystemExit("Invalid map-to-odom seed")
anchor = seed.get("anchor_ns", 0)
if type(anchor) is not int or not 0 <= anchor < 2**32 * 10**9:
    raise SystemExit("Invalid seed anchor")
print(json.dumps(values))
print(anchor)
PY
)
seed_values=${seed_info%$'\n'*}
anchor=${seed_info##*$'\n'}
printf '%s\n' "$seed_values" >"$output/applied_seed.json"
printf '%s\n' "$anchor" >"$output/seed_anchor_ns.txt"
sha256sum "$bag" "$map/map_world.pcd" >"$output/inputs.sha256"

# Serialize local DLL replays and check for an unrelated process on the requested port.
exec 9>"/tmp/fastlio-dll-replay-$port.lock"
flock -n 9
python3 - "$port" <<'PY'
import socket
import sys

with socket.socket() as sock:
    sock.bind(("127.0.0.1", int(sys.argv[1])))
PY

master_pid=
launch_pid=
record_pid=
player_pid=
fixture_pid=
cleanup() {
  for pid in "$player_pid" "$record_pid" "$fixture_pid" "$launch_pid" "$master_pid"; do
    if [[ -n "$pid" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for() {
  local limit=$1
  shift
  local start=$SECONDS
  until "$@"; do
    if (( SECONDS - start >= limit )); then
      echo "Replay readiness check timed out: $*" >&2
      return 1
    fi
    sleep 0.25
  done
}
master_ready() { kill -0 "$master_pid" 2>/dev/null && timeout 2 rosnode list >/dev/null 2>&1; }
node_ready() { timeout 2 rosnode ping -c 1 "$1" >/dev/null 2>&1; }
roscore -p "$port" >"$output/master.log" 2>&1 &
master_pid=$!
wait_for 20 master_ready

standalone=true
if (( observe )); then standalone=false; fi
roslaunch fastlio_dll replay.launch "map_pcd:=$map/map_world.pcd" "start_standalone_dll:=$standalone" \
  "seed_map_odom:=$seed_values" "seed_not_before_ns:=$anchor" \
  ${DLL_REPLAY_CONFIG:+"config:=$DLL_REPLAY_CONFIG"} >"$output/launch.log" 2>&1 &
launch_pid=$!
wait_for 15 node_ready /laserMapping
wait_for 15 node_ready /gravity_bridge

result_dir=$output
if (( observe )); then
  invalidate_at=$(python3 - "$bag" <<'PY'
import sys
import rosbag
with rosbag.Bag(sys.argv[1]) as bag:
    print(bag.get_end_time() - bag.get_start_time() - 4.0)
PY
)
  python3 "$(dirname "$0")/replay_legacy_seed.py" "$map" "$seed_values" "$invalidate_at" \
    >"$output/legacy_fixture.log" 2>&1 &
  fixture_pid=$!
  wait_for 15 node_ready /dll_replay_legacy_seed
  result_dir="$output/observer"
  "$(dirname "$0")/record_observation.sh" "$map" "$result_dir" >"$output/observer.log" 2>&1 &
  record_pid=$!
  recorder_ready() { test -s "$result_dir/rosparams.yaml" && kill -0 "$record_pid"; }
  wait_for 70 recorder_ready
else
  # Node registration occurs before its distance field is ready.
  timeout "${DLL_STARTUP_TIMEOUT:-60}" rostopic echo -n 1 /dll_localization/status >"$output/startup_status.yaml"
  timeout 5 rosparam dump "$output/rosparams.yaml" /fastlio_dll
  rosbag record __name:=dll_record -O "$output/result.bag" \
    /fastlio/odom_world /dll_localization/pose /dll_localization/correction \
    /dll_localization/status >"$output/record.log" 2>&1 &
  record_pid=$!
  wait_for 15 node_ready /dll_record
fi
dll_pid=$(rosnode info /fastlio_dll | awk '/^Pid:/ {print $2}')

play_options=()
if [[ -n "${DLL_REPLAY_SECONDS:-}" ]]; then
  play_options+=(--duration="$DLL_REPLAY_SECONDS")
fi
rosbag play --clock --rate=1 --delay=1 "${play_options[@]}" "$bag" --topics /livox/lidar /livox/imu \
  >"$output/play.log" 2>&1 &
player_pid=$!
wait "$player_pid"
player_pid=
sleep 0.75
kill -0 "$launch_pid"
awk '/^VmHWM:/ {print $2}' "/proc/$dll_pid/status" >"$result_dir/dll_peak_rss_kib.txt"
if (( observe )); then
  kill -TERM "$record_pid"
  rc=0
  wait "$record_pid" || rc=$?
  if [[ "$rc" != 143 ]]; then exit 1; fi
else
  kill -INT "$record_pid"
  wait "$record_pid"
fi
record_pid=
if (( observe )); then
  python3 "$(dirname "$0")/summarize_replay.py" "$result_dir" --verify-observation-replay
else
  python3 "$(dirname "$0")/summarize_replay.py" "$result_dir"
fi
