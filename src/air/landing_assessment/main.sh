#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${UAV_ENV:-${VIRTUAL_ENV:-$HOME/uav_prediction_env}}/bin/activate"
exec python main.py "$@"
