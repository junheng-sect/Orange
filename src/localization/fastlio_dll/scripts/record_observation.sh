#!/usr/bin/env bash
# Join the ROS master selected by the caller. Own only the observer and its recorder.
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 MAP_DIR NEW_OUTPUT_DIR [DLL_CONFIG]" >&2
  exit 2
fi
map_dir=$(realpath -e "$1")
output=$(realpath -m "$2")
package=$(rospack find fastlio_dll)
config=$(realpath -e "${3:-$package/config/indoor.yaml}")
test -r "$map_dir/map_world.pcd"

exec 9>"${TMPDIR:-/tmp}/fastlio-dll-observation-$(id -u).lock"
flock -n 9 || { echo "An observation recorder is already running" >&2; exit 1; }
nodes=$(timeout 5 rosnode list)
if grep -Eq '^/(fastlio_dll|dll_observation_record)$' <<<"$nodes"; then
  echo "An observer or its recorder is already registered on this ROS master" >&2
  exit 1
fi

mkdir -p "$(dirname "$output")"
mkdir "$output"
sha256sum "$map_dir/map_world.pcd" >"$output/map.sha256"
cp "$config" "$output/config.yaml"
export ROS_LOG_DIR="$output/roslog"

record_pid=
launch_pid=
cleanup() {
  for pid in "$launch_pid" "$record_pid"; do
    if [[ -n "$pid" ]]; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

rosbag record __name:=dll_observation_record -O "$output/observation.bag" \
  /livox/lidar /livox/imu /Odometry /fastlio/gravity_init /fastlio/odom_world /fastlio/velocity \
  /fastlio/vision_pose_cov /fastlio/vision_odom \
  /drone_reloc/result /drone_reloc/status /drone_reloc/pose /tf /tf_static \
  /dll_localization/pose /dll_localization/correction /dll_localization/status \
  /mavros/state /mavros/extended_state /mavros/estimator_status /mavros/local_position/odom \
  >"$output/record.log" 2>&1 &
record_pid=$!
deadline=$((SECONDS + 15))
until timeout 2 rosnode ping -c 1 /dll_observation_record >/dev/null 2>&1; do
  if ! kill -0 "$record_pid" 2>/dev/null || (( SECONDS >= deadline )); then
    echo "Recorder did not start. See $output/record.log" >&2
    exit 1
  fi
  sleep 0.25
done

roslaunch fastlio_dll observe.launch "map_dir:=$map_dir" "config:=$output/config.yaml" \
  >"$output/observe.log" 2>&1 &
launch_pid=$!
timeout 60 rostopic echo -n 1 /dll_localization/status >"$output/startup_status.yaml"
timeout 5 rosparam dump "$output/rosparams.yaml" /fastlio_dll
echo "Recording $output/observation.bag. Ctrl+C closes the bag."
wait -n "$launch_pid" "$record_pid" || true
echo "Observer or recorder exited. See $output/observe.log and $output/record.log" >&2
exit 1
