#!/usr/bin/env bash
set -euo pipefail

# Run this on SEEKER after the preview launch is already running.
# The timeout sends SIGINT so rosbag can close the bag cleanly.

DURATION_SEC="${1:-60}"
workspace="$(cd -- "$(dirname -- "$(realpath -e -- "$0")")/.." && pwd -P)"
BASE_DIR="${2:-$workspace/logs/front_marker_jitter}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${BASE_DIR}/${STAMP}"
mkdir -p "${OUT_DIR}"

TOPICS=(
  /front_marker/pose
  /front_marker_control/position_error
  /front_marker_control/yaw_error
  /front_marker_control/cmd_vel
  /front_marker_control/setpoint_preview
  /front_marker_control/status
)

echo "capture_dir=${OUT_DIR}"
echo "duration_sec=${DURATION_SEC}"
echo "setpoint_output=false is required from the launch command"

set +e
timeout --signal=INT --kill-after=5 "${DURATION_SEC}s" \
  rosbag record -O "${OUT_DIR}/capture.bag" "${TOPICS[@]}" \
  >"${OUT_DIR}/capture.log" 2>&1
RC=$?
set -e

if [[ "${RC}" -ne 0 && "${RC}" -ne 124 && "${RC}" -ne 130 && "${RC}" -ne 143 ]]; then
  echo "rosbag record failed with exit code ${RC}" >&2
  exit "${RC}"
fi

test -s "${OUT_DIR}/capture.bag"
echo "capture_bag=${OUT_DIR}/capture.bag"
