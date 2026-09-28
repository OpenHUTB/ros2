#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source "${UAV_ENV:-$HOME/uav_prediction_env}/bin/activate"
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
exec python main.py "$@"
