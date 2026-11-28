#!/usr/bin/env bash
# ==============================================================================
# 室内原地起降：POSCTL 地面人工解锁，自动起飞、定点悬停、慢降至着地上锁，仅一趟。
# --height M   相对起飞高度，默认 0.50 m，另受室内地图限高和导航边界约束
# --hover S    停稳 2 s 后额外悬停，默认 3 s
# --delay S    人工解锁后倒计时，默认 3 s
# --dry-run    只校验并打印本趟参数
# ==============================================================================
TEST_NAME=test_indoor_takeoff_land
SCRIPTS=$(cd "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")" && pwd)
source "$SCRIPTS/nav_test/common.sh"
DEFAULT_MAP="$MAP_ROOT/20261114_101654_indoor_fly_raw_000001"
DEFAULT_NAV_CONFIG="$NT_DIR/indoor_fly_nav.yaml"
DEFAULT_RELOC_CONFIG="$WS/src/localization/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$WS/src/localization/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_EV_MODE=odom
flight_args=()
common_args=()
dry=0
while (( $# )); do
  case "$1" in
    --height|--hover|--delay)
      (( $# >= 2 )) || { echo "$1 缺少数值" >&2; exit 2; }
      flight_args+=("$1" "$2"); shift 2 ;;
    --no-dll|--record-raw-lidar|--record-nav-cloud|--allow-unsynced-clock|--show-dll-config)
      common_args+=("$1"); shift ;;
    --dry-run) dry=1; shift ;;
    -h|--help) sed -n '3,7s/^# //p' "$0"; exit 0 ;;
    *) echo "未知或不适用于室内原地起降的选项：$1" >&2; exit 2 ;;
  esac
done
values=$(python3 -B "$NT_DIR/vertical_profile.py" "${flight_args[@]}") || exit 2
read -r HEIGHT HOVER DELAY <<<"$values"
test_desc() { echo "室内原地起飞 ${HEIGHT} m，停稳后悬停 ${HOVER} s，人工解锁倒计时 ${DELAY} s"; }
if (( dry )); then test_desc; exit 0; fi
# Keep these arguments when common.sh hands the original command to tmux.
test_arg() { case "$1" in --height|--hover|--delay) SHIFT=2 ;; *) return 1 ;; esac; }
test_preflight() { python3 -B "$NT_DIR/nav_indoor3.py" --verify-map "$MAP"; }
run_test() {
  nt_check_chain || return 1
  python3 -B "$NT_DIR/nav_indoor_takeoff_land.py" --out "$OUT" --height "$HEIGHT" --hover "$HOVER" --delay "$DELAY" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
}
nav_test_main "${common_args[@]}" "${flight_args[@]}"
