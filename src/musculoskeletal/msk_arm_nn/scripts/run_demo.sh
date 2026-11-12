#!/usr/bin/env bash
# MSK-Arm desktop demo launcher.
#
#   scripts/run_demo.sh [mode] [scene] [extra roslaunch args...]
#
#   mode   teleop | reach | perception | e2e | benchmark | bringup
#   scene  reach | shelf | draw
#
# Examples:
#   scripts/run_demo.sh                        # keyboard teleoperation, reach
#   scripts/run_demo.sh reach shelf            # muscle-synergy reaching on shelf
#   scripts/run_demo.sh benchmark draw duration:=60
#
# The viewer window is opened on the real desktop (DISPLAY=:0) because the
# course demo video is recorded from it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${MSKWS:-$HOME/msk_ws}"
MODE="${1:-teleop}"
SCENE="${2:-reach}"
if [ $# -ge 1 ]; then shift; fi
if [ $# -ge 1 ]; then shift; fi

export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ] || [ ! -f "${XAUTHORITY:-/nonexistent}" ]; then
  for cand in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
    if [ -f "$cand" ]; then
      export XAUTHORITY="$cand"
      break
    fi
  done
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ROS Noetic not found at /opt/ros/noetic" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/devel/setup.bash"
else
  echo "catkin workspace not built yet: $WS" >&2
  echo "Run: bash $ROOT/scripts/build_ros.sh $WS" >&2
  exit 1
fi

echo "[run_demo] mode=$MODE scene=$SCENE display=$DISPLAY"
exec roslaunch msk_arm_nn "${MODE}.launch" "scene:=${SCENE}" "$@"
