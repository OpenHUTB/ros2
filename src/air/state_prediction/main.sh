#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -r "$ros_setup" ]; then echo "ROS2 setup not found: $ros_setup. Set ROS_SETUP." >&2; exit 2; fi
source "$ros_setup"
course_env="${UAV_ENV:-${VIRTUAL_ENV:-$HOME/uav_prediction_env}}"
if [ ! -r "$course_env/bin/activate" ]; then echo "Python environment not found: $course_env. Follow README environment setup." >&2; exit 2; fi
source "$course_env/bin/activate"
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
exec python main.py "$@"
