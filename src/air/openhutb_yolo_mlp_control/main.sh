#!/usr/bin/env bash
# Source ROS 2 and the workspace overlay before running this entry point.
set -e
exec ros2 launch openhutb_yolo_mlp_control visual_centering.launch.py "$@"
