#!/usr/bin/env bash
# Build the RVNN catkin package in an isolated workspace (does not touch any
# pre-existing catkin workspace of the machine).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/roscx_ws}"
if [ ! -d /opt/ros/noetic ]; then
  echo "ROS Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi
mkdir -p "$WS/src"
ln -sfn "$ROOT" "$WS/src/rov_numpy_nn_slam"
cd "$WS"
# ROS setup scripts reference unset variables, so relax `set -u` while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u
catkin_make -j2
cat <<MSG

Build finished.
  source $WS/devel/setup.bash
  roslaunch rov_numpy_nn_slam teleop.launch
MSG
