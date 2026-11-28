#!/usr/bin/env bash
# Run inside the built Noetic workspace. Synthetic ROS tests only use loopback.
set -eo pipefail
workspace="$(cd -- "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")/../.." && pwd -P)"
cd "$workspace"
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

status=0
check() {
    printf '\n>>> %s\n' "$*"
    if ! "$@"; then
        printf 'FAILED: %s\n' "$*" >&2
        status=1
    fi
}

# Keep timing-sensitive rostests independent of other packages' compilations.
check catkin test --continue-on-failure -p1 --no-status --make-args -j1
check catkin_test_results build

# These existing suites are not registered in their package's CMakeLists.txt.
for package in car_bridge drone_mapping drone_relocalization fastlio_dll; do
    check python3 -m unittest discover -s "src/localization/$package/test" -p 'test_*.py'
done
check python3 src/localization/ywl_localization/test/test_init_adapter.py
for suite in land takeoff; do
    check python3 -m unittest discover -s src/navigation/drone_navigation/test -p "test_$suite.py"
done
check python3 -m unittest discover -s scripts/nav_test/tests -p 'test_nav_*.py'
check python3 -m unittest discover -s scripts/nav_test/tests -p test_startup_optimizations.py
check python3 -m unittest discover -s scripts/nav_test/tests -p test_indoor3_3_lock.py
check bash scripts/nav_test/tests/test_dll_defaults.sh
check python3 scripts/rc_record/test_rc_record_policy.py
check python3 -m unittest discover -s scripts/rc_record -p test_recorder_topics.py
check python3 scripts/rc_record/test_raw_bag_recorder_e2e.py
check python3 scripts/nav_test/tests/test_indoor3_4_video.py
check python3 scripts/nav_test/tests/test_indoor3_4_video.py --dual
for suite in test_dll_drift.py test_ywl_monitor.py test_board_tasks.py test_mission_presets.py test_scan_records.py; do
    check python3 -m unittest discover -s tools/warehouse_monitor -p "$suite"
done
check python3 -m unittest discover -s tools/car_web -p 'test_*.py'
exit "$status"
