#!/usr/bin/env bash
# Usage: bash scripts/run_navigation.sh SIM_PYTHON ASSETS [SEED]
set -e
MODULE_DIR="$(cd -- "$(dirname -- "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11331}"
export ROS_PACKAGE_PATH="$MODULE_DIR:$ROS_PACKAGE_PATH"
SIM_PYTHON="${1:?Provide the simulator Python executable}"
ASSETS="${2:?Provide the unitree_rl_gym path}"
ENTRY="$MODULE_DIR/main.py"
if [[ "$SIM_PYTHON" == *.exe ]]; then
    ENTRY="$(wslpath -w "$ENTRY")"
    ASSETS="$(wslpath -w "$ASSETS")"
fi
exec roslaunch -p "${ROS_MASTER_URI##*:}" g1_gmapping_navigation main.launch \
    sim_python:="$SIM_PYTHON" sim_entry:="$ENTRY" assets:="$ASSETS" seed:="${3:-7}"
