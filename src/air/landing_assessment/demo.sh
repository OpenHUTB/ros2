#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source install/setup.bash
ros2 launch landing_assessment main.launch.py model:="$PWD/models/trained/mlp_42" files:="${1:-$PWD/data/raw/episode_1096.npz;$PWD/data/raw/episode_1097.npz;$PWD/data/raw/episode_1098.npz}" log:="$PWD/results/ros_predictions.jsonl"
