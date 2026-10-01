#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "$ROS_DISTRO" ]; then source /opt/ros/noetic/setup.bash; fi
if command -v conda &> /dev/null; then
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate carla38
fi
cd "$SCRIPT_DIR"
exec roslaunch carla_pilotnet_ros main.launch "$@"
