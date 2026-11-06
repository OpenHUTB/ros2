#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source install/setup.bash
ros2 launch payload_mass_estimation main.launch.py project_root:="$PWD"
