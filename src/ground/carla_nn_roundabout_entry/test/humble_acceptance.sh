#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../../../.." && pwd)"
cd "$repo_root"
colcon build --packages-select carla_nn_roundabout_entry --symlink-install
source install/setup.bash
colcon test --packages-select carla_nn_roundabout_entry --event-handlers console_direct+
colcon test-result --verbose

timeout --signal=INT --kill-after=5s 28s \
  ros2 launch carla_nn_roundabout_entry main.launch.py \
    metrics_path:=/tmp/roundabout_navigation_metrics.json
test -s /tmp/roundabout_navigation_metrics.json
cat /tmp/roundabout_navigation_metrics.json
