#!/usr/bin/env bash
# Replay a complete handheld A-B-A bag on the board's isolated ROS masters.
set -eo pipefail
if [[ $# -lt 2 || "$1" == --help ]]; then
  echo "Usage: $0 RAW_BAG NEW_OUTPUT_DIR [--b-at SECONDS] [--b-hint X Y Z IMU_YAW_DEG]"
  [[ ${1:-} == --help ]] && exit 0
  exit 2
fi
bag=$(realpath -e "$1")
out=$(realpath -m "$2")
shift 2
dir=$(dirname "$(realpath -e "$0")")
original="$HOME/test_data/mapping/20261105_114605_outdoor"
map=${ABA_MAP_DIR:-${original}_screenshot_roi}
export PAIR_SEED_MAP=${PAIR_SEED_MAP:-$original}
export DLL_REPLAY_CONFIG=${DLL_REPLAY_CONFIG:-$dir/../config/outdoor_large_field.yaml}
export DLL_STARTUP_TIMEOUT=${DLL_STARTUP_TIMEOUT:-240}
export PAIR_ALLOW_EMPTY_YWL=1
unset PAIR_REPLAY_SECONDS DLL_REPLAY_SECONDS
"$dir/replay_pair_outdoor.sh" "$bag" "$out" "$map"
source "$HOME/mid360s_ws/devel/setup.bash"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 "$dir/analyze_aba.py" "$out" "$@"
