#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${UAV_ENV:-$HOME/uav_prediction_env}/bin/activate"
case "${1:-help}" in
  build) python -m colcon build --base-paths . --packages-select uav_telemetry_diagnostics ;;
  train) python train_consistency.py ;;
  test) python -m unittest discover -s tests -v ;;
  report) python report_final.py ;;
  demo)
    source install/setup.bash
    mkdir -p results
    scenario="${2:-sample/observations.csv}"
    python -c 'from ros_diagnostics import diagnostics_main; diagnostics_main()' --ros-args -p model:="$PWD/models/consistency_42" -p log:="$PWD/results/ros_diagnostics.jsonl" &
    detector_pid=$!
    trap 'kill -INT "$detector_pid" 2>/dev/null || true; wait "$detector_pid" 2>/dev/null || true' EXIT
    timeout --signal=INT 28s ros2 run uav_telemetry_diagnostics telemetry_replay --ros-args -p csv:="$PWD/$scenario" || test "$?" -eq 124
    ;;
  *) echo 'Usage: bash main.sh {build|train|test|report|demo [relative-observations.csv]}' ;;
esac
