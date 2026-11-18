#!/usr/bin/env bash
# Run seed, DLL and YWL sequentially in independent ROS masters.
set -eo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 RAW_BAG NEW_OUTPUT_DIR [SHARED_MAP_DIR]" >&2
  exit 2
fi
bag=$(realpath -e "$1")
out=$(realpath -m "$2")
map=$(realpath -e "${3:-$HOME/test_data/mapping/20261105_114605_outdoor_shared}")
ywl_ws=${YWL_REPLAY_WS:-$HOME/mid360s_ws-ywl}
dll_ws=${DLL_REPLAY_WS:-$HOME/test_data/fastlio_dll/20261117/outdoor_ws}
seed_seconds=${PAIR_SEED_SECONDS:-18}
seed_map=$(realpath -e "${PAIR_SEED_MAP:-$map}")
scripts=$(cd "$(dirname "$0")" && pwd)

test -s "$ywl_ws/devel/setup.bash"
test -s "$dll_ws/devel/setup.bash"
test -s "$map/outdoor_reloc.yaml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ywl_play_options=()
if [[ -n "${PAIR_REPLAY_SECONDS:-}" ]]; then
  ywl_play_options+=(--seconds "$PAIR_REPLAY_SECONDS")
fi
mkdir -p "$(dirname "$out")"
mkdir "$out"
(
  source "$ywl_ws/devel/setup.bash"
  # A flight bag can contain PX4 messages before the LiDAR and IMU have started.
  # In that case anchor the seed replay window to the first raw sensor data.
  seed_bag=$(python3 - "$bag" "$out" "$seed_seconds" <<'PY'
from pathlib import Path
import sys

import rosbag
import rospy

bag, out, seconds = sys.argv[1:]
topics = ["/livox/lidar", "/livox/imu"]
with rosbag.Bag(bag) as source:
    first = next(source.read_messages(topics=topics), None)
    if first is None:
        raise SystemExit("bag contains no raw LiDAR or IMU")
    if first.timestamp.to_sec() - source.get_start_time() > 0.5:
        path = Path(out) / "seed_raw_prefix.bag"
        end = first.timestamp + rospy.Duration.from_sec(float(seconds))
        with rosbag.Bag(str(path), "w") as target:
            for topic, msg, stamp in source.read_messages(topics=topics, end_time=end):
                target.write(topic, msg, stamp)
        print(path)
    else:
        print(bag)
PY
)
  rosrun ywl_localization ywl_replay seed --map "$seed_map" --bag "$seed_bag" --out "$out/seed" \
    --reloc-config "$map/outdoor_reloc.yaml" --port "${PAIR_SEED_PORT:-11590}" \
    --rate 0.5 --seconds "$seed_seconds"
)

python3 - "$out/seed/seed.json" "$map" "$bag" "$out/dll_seed.json" "$seed_map" "$out/ywl_seed.json" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

seed_path, map_dir, bag_path, output, seed_map, ywl_output = map(Path, sys.argv[1:])
seed = json.loads(seed_path.read_text())

def hashes(folder):
    return {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
            for name in ("map_world.pcd", "meta.json")}

origin = hashes(seed_map)
if seed["map_sha256"] != origin:
    raise SystemExit("Seed map changed since initialization")
target = origin
if seed_map != map_dir:
    target = hashes(map_dir)
    crop = json.loads((map_dir / "meta.json").read_text())["shared_outdoor_crop"]
    source = crop["source"]
    if (crop["frame_id"] != "drone_map" or source["map_sha256"] != origin["map_world.pcd"]
            or source["meta_sha256"] != origin["meta.json"]):
        raise SystemExit("Tracking crop does not share the initialization map frame")
result = seed["result"]
if result.get("confirmed") is not True or result.get("ok") is not True:
    raise SystemExit("Relocalization seed is not confirmed")
seed.update(map_sha256=target, initialization_map=str(seed_map), initialization_map_sha256=origin)
ywl_output.write_text(json.dumps(seed, indent=2) + "\n")
output.write_text(json.dumps(dict(bag=str(bag_path), map=str(map_dir), map_sha256=target,
                                  initialization_map_sha256=origin, anchor_ns=seed["anchor_ns"],
                                  results=[result]), indent=2) + "\n")
PY

(
  source "$dll_ws/devel/setup.bash"
  DLL_REPLAY_PORT=${DLL_REPLAY_PORT:-11471} DLL_REPLAY_SECONDS=${PAIR_REPLAY_SECONDS:-} \
    "$scripts/replay.sh" "$bag" "$map" "$out/dll" "$out/dll_seed.json"
)
(
  source "$ywl_ws/devel/setup.bash"
  rosrun ywl_localization ywl_replay ywl --map "$map" --bag "$bag" --out "$out/ywl" \
    --seed "$out/ywl_seed.json" --port "${PAIR_YWL_PORT:-11591}" "${ywl_play_options[@]}"
)
if [[ ! -s "$out/ywl/pose.csv" ]] || [[ $(wc -l <"$out/ywl/pose.csv") -le 1 ]]; then
  echo "YWL did not publish map poses. See $out/ywl/diagnostics.jsonl" >&2
  [[ "${PAIR_ALLOW_EMPTY_YWL:-0}" == 1 ]] || exit 1
fi
