#!/usr/bin/env bash
# RVNN desktop demo launcher.
#
#   scripts/run_demo.sh [mode] [scene] [extra roslaunch args...]
#
#   mode   teleop | slam_nav | perception | e2e | benchmark | bringup
#   scene  pool | harbor | cave
#
# Examples:
#   scripts/run_demo.sh                 # keyboard teleoperation in the pool
#   scripts/run_demo.sh slam_nav cave   # SLAM + A* mission in the cave
#   scripts/run_demo.sh benchmark pool duration:=120
#
# The viewer window is opened on the real desktop (DISPLAY=:0) because the
# course demo video is recorded from it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROVNN_WS:-$HOME/roscx_ws}"
MODE="${1:-teleop}"
SCENE="${2:-pool}"
if [ $# -ge 1 ]; then shift; fi
if [ $# -ge 1 ]; then shift; fi

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ROS Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/devel/setup.bash"
else
  echo "catkin workspace not built yet: $WS" >&2
  echo "Run: bash $ROOT/scripts/build_ros.sh $WS" >&2
  exit 1
fi

echo "[run_demo] mode=$MODE scene=$SCENE display=$DISPLAY"
exec roslaunch rov_numpy_nn_slam "${MODE}.launch" "scene:=${SCENE}" "$@"
