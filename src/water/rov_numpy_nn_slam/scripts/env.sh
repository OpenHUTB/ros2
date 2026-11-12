#!/usr/bin/env bash
# RVNN runtime environment for interactive shells and the recorded demo.
# Sources ROS Noetic and the isolated catkin workspace; nothing is persisted in
# ~/.bashrc or any other system file.

export ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RVNN_WS="${RVNN_WS:-$HOME/roscx_ws}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "$RVNN_WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$RVNN_WS/devel/setup.bash"
fi

# Make the project importable outside catkin as well (tools/tests).
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
export RVNN_VENV="$ROOT/.venv"
