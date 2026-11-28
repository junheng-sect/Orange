#!/usr/bin/env bash
# The public CLI passes a validated, frozen mission before acquiring the shared flight lock.
source "$INSPECTION_WORKSPACE/scripts/nav_test/common.sh"
NT_RUN_SUFFIX=$INSPECTION_SUFFIX
NT_PIPELINE=0
[[ $TEST_NAME != mission_indoor_shelf ]] || NT_PIPELINE=1
NT_GROUND_CLEANUP=1
NT_TMP_TASK_LOG=1
test_navigation_args() { _nav_args+=(allow_short_goals:=true); }
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3
test_desc() { echo "巡检 $TEST_NAME"; }
test_arg() { return 1; }

test_setup_runtime() {
  export INSPECTION_LIVE="$NT_LOG_DIR" INSPECTION_ARCHIVE="$OUT"
  export INSPECTION_PREVIEW="$PREVIEW"
  NAV_CONFIG="$RUN_DIR/navigation.yaml"
  MAP_FRAME="$RUN_DIR/frames.yaml"
  NT_DLL=$INSPECTION_DLL
  python3 -m drone_inspection.runtime setup "$RUN_DIR" || return 1
  [ -f "$MAP_FRAME" ] || MAP_FRAME=""
}
test_preflight_args() {
  if (( NT_PIPELINE )); then _pf_args+=(--managed-fusion); fi
  _pf_args+=(--expected-sha "$INSPECTION_MAP_SHA")
  if (( INSPECTION_CAMERA )); then
    _pf_args+=(--forbid-topics-regex '(/ov13855(_full)?/image_raw|/ov13855_apriltag_vision/(corners|detections)|/front_marker/pose|/barcode_reader_camera/result|/drone_inspection/target_tag_pose)')
  fi
}
test_bringup() {
  (( INSPECTION_CAMERA )) || return 0
  log "启动巡检相机（日志 $NT_LOG_DIR/camera.log）"
  PF_SCAN_CAMERA="$RUN_DIR/inspection_camera.pid"
  start_child "$PF_SCAN_CAMERA" "$NT_LOG_DIR/camera.log" roslaunch drone_inspection shelf_camera.launch save_dir:="$OUT/barcode" save_frames:="$INSPECTION_FRAMES"
  child_pid "$PF_SCAN_CAMERA" >/dev/null || return 1
  if (( NT_PIPELINE )); then
    PF_CAM_PROBE="$RUN_DIR/camera_probe.pid"
    start_child "$PF_CAM_PROBE" "$NT_LOG_DIR/camera_probe.log" python3 -u -m drone_inspection.runtime probe
    CAM_JOB=$NT_CHILD_JOB
  fi
}
test_after_fusion() {
  (( INSPECTION_CAMERA )) || return 0
  log "检查巡检相机双路图像与读码服务"
  local TIMEFORMAT='相机 probe 进程总耗时 %3R s'
  { time python3 -m drone_inspection.runtime probe; } 2>&1 | tee -a "$LOG"
}
test_warm_runner() {
  PF_WARM="$RUN_DIR/runner.pid"
  start_child "$PF_WARM" "$NT_LOG_DIR/runner.log" bash -c 'exec python3 -u -m drone_inspection.runtime warm "$1" > >(exec tee -a "$2" >&3) 2>&1' _ "$RUN_DIR/release" "$LOG" 3>&1
  WARM_JOB=$NT_CHILD_JOB
}
run_test() {
  nt_check_chain || return 1
  python3 -u -m drone_inspection.runtime run > >(trap '' INT; exec tee -a "$LOG") 2>&1
}
test_cleanup() {
  local rc=0
  if [[ -n ${PF_WARM:-} ]]; then stop_child "$PF_WARM" "巡检运行器" TERM 8 || rc=1; fi
  if [[ -n ${PF_CAM_PROBE:-} ]]; then stop_child "$PF_CAM_PROBE" "巡检相机探针" TERM 5 || rc=1; fi
  if [[ -n ${PF_SCAN_CAMERA:-} ]]; then stop_child "$PF_SCAN_CAMERA" "巡检相机" INT 10 || rc=1; fi
  python3 -m drone_inspection.runtime finalize "${NT_LAST_ERROR:-}" || rc=1
  return "$rc"
}
test_after_cleanup() { rm -f -- "$OUT/.inspection_live"; }

nav_test_main "$@"
