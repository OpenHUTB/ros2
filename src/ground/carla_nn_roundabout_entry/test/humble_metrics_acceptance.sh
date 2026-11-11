#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
colcon build --packages-select carla_nn_roundabout_entry --symlink-install
source install/setup.bash
colcon test --packages-select carla_nn_roundabout_entry --event-handlers console_direct+
colcon test-result --verbose
python3 -m carla_nn_roundabout_entry.evaluate_scenarios \
  --seeds 3 7 11 19 23 --steps 200 --controller both \
  > /tmp/roundabout-metrics.json
grep -q '"scenario_count": 5' /tmp/roundabout-metrics.json
grep -q '"image-model"' /tmp/roundabout-metrics.json
