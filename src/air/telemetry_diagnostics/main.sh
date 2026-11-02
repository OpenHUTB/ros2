#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -r "$ros_setup" ]; then echo "ROS2 setup not found: $ros_setup. Set ROS_SETUP." >&2; exit 2; fi
source "$ros_setup"
course_env="${UAV_ENV:-${VIRTUAL_ENV:-$HOME/uav_prediction_env}}"
if [ ! -r "$course_env/bin/activate" ]; then echo "Python environment not found: $course_env. Follow README environment setup." >&2; exit 2; fi
source "$course_env/bin/activate"
case "${1:-help}" in
  doctor) python main.py doctor ;;
  build) python main.py build ;;
  train) python train_consistency.py ;;
  test) python main.py test ;;
  report) python report_final.py ;;
  demo)
    if [ ! -f install/setup.bash ]; then python main.py build; fi
    source install/setup.bash
    mkdir -p results
    scenario="${2:-sample/observations.csv}"
    python -c 'from ros_diagnostics import diagnostics_main; diagnostics_main()' --ros-args -p model:="$PWD/models/consistency_42" -p log:="$PWD/results/ros_diagnostics.jsonl" &
    detector_pid=$!
    trap 'kill -INT "$detector_pid" 2>/dev/null || true; wait "$detector_pid" 2>/dev/null || true' EXIT
    timeout --signal=INT 28s ros2 run telemetry_diagnostics telemetry_replay --ros-args -p csv:="$PWD/$scenario" || test "$?" -eq 124
    ;;
  *) echo 'Usage: bash main.sh {build|train|test|report|demo [relative-observations.csv]}' ;;
esac
