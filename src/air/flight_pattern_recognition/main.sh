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
  test) python main.py test ;;
  demo)
    if [ ! -f data/prepared/final.npz ]; then python main.py prepare; fi
    if [ ! -f install/setup.bash ]; then python main.py build; fi
    source install/setup.bash
    files="${2:-$PWD/data/final_raw/episode_300.csv;$PWD/data/final_raw/episode_301.csv;$PWD/data/final_raw/episode_302.csv;$PWD/data/final_raw/episode_303.csv;$PWD/data/final_raw/episode_304.csv}"
    ros2 launch flight_pattern_recognition pattern.launch.py files:="$files" model:="$PWD/models/mlp_42" log:="$PWD/results/ros_predictions.jsonl"
    ;;
  dashboard) python main.py dashboard ;;
  *) echo 'Usage: bash main.sh {build|test|demo [semicolon-separated CSV paths]|dashboard}' ;;
esac
