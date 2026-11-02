#!/usr/bin/env bash
# Build the package in an isolated catkin workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/gvnn_ws}"
if [ ! -d /opt/ros/noetic ]; then
  echo "ROS Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi
mkdir -p "$WS/src"
ln -sfn "$ROOT" "$WS/src/mujoco_ground_slam_nn_nav"
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
catkin_make -j2
cat <<MSG

Build finished.
  source $WS/devel/setup.bash
  roslaunch mujoco_ground_slam_nn_nav teleop.launch
MSG
