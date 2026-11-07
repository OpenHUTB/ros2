#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null
ros2 run bezier_trajectory main "$@"
