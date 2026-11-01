#!/usr/bin/env bash
# Source ROS 2 and the workspace overlay before running this entry point.
set -e
exec ros2 launch uav_spiral_search spiral_search.launch.py "$@"
