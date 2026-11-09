#!/usr/bin/env bash
# Usage: bash scripts/run_navigation.sh SIM_PYTHON ASSETS [TASK] [WEIGHTS] [OUTPUT]
set -e
MODULE_DIR="$(cd -- "$(dirname -- "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11331}"
export ROS_PACKAGE_PATH="$MODULE_DIR:$ROS_PACKAGE_PATH"
SIM_PYTHON="${1:?Provide the simulator Python executable}"
ASSETS="${2:?Provide the unitree_rl_gym path}"
TASK="${3:-navigation}"
WEIGHTS="${4:-$MODULE_DIR/models}"
OUTPUT="${5:-$MODULE_DIR/results}"
ENTRY="$MODULE_DIR/main.py"
if [[ "$SIM_PYTHON" == *.exe ]]; then
    ENTRY="$(wslpath -w "$ENTRY")"
    ASSETS="$(wslpath -w "$ASSETS")"
    WEIGHTS="$(wslpath -w "$WEIGHTS")"
    OUTPUT="$(wslpath -w "$OUTPUT")"
fi
exec roslaunch -p "${ROS_MASTER_URI##*:}" humanoid_visual_patrol main.launch \
    sim_python:="$SIM_PYTHON" sim_entry:="$ENTRY" assets:="$ASSETS" \
    task:="$TASK" weights:="$WEIGHTS" output:="$OUTPUT"
