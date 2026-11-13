#!/usr/bin/env bash
# Build the GRNN road-vehicle catkin package in an isolated workspace (does not
# touch any pre-existing catkin workspace of the machine).
#
#   bash scripts/build_ros.sh [workspace]      # default: $HOME/road_ws
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$HOME/road_ws}"
if [ ! -d /opt/ros/noetic ]; then
  echo "ROS Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi

# roslaunch resolves node types from the source tree and requires the exec bit,
# which an archive copy on a Windows host does not preserve.
chmod +x "$ROOT"/scripts/*.py "$ROOT"/scripts/*.sh 2>/dev/null || true

mkdir -p "$WS/src"
ln -sfn "$ROOT" "$WS/src/road_numpy_nn_slam"
cd "$WS"
# ROS setup scripts reference unset variables, so relax `set -u` while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u
catkin_make -j2
cat <<MSG

Build finished ($WS).
  source /opt/ros/noetic/setup.bash
  source $WS/devel/setup.bash
  roslaunch road_numpy_nn_slam teleop.launch scene:=downtown
MSG
