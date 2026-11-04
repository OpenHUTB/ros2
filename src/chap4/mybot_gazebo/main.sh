#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash

for setup_file in \
  "${SCRIPT_DIR}/../../devel/setup.bash" \
  "${SCRIPT_DIR}/../../../devel/setup.bash" \
  "${HOME}/mybot_ws/devel/setup.bash" \
  "${HOME}/catkin_ws/devel/setup.bash"
do
  if [ -f "${setup_file}" ]; then
    source "${setup_file}"
    break
  fi
done

if ! rospack find mybot_gazebo >/dev/null 2>&1; then
  echo "mybot_gazebo is not found in the current ROS environment." >&2
  echo "Build the package first, then source the workspace setup.bash." >&2
  exit 1
fi

exec roslaunch mybot_gazebo mybot_gazebo.launch "$@"
