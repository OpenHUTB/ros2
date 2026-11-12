#!/usr/bin/env bash
# HNN humanoid runtime environment for interactive shells and the demo.
# Sources ROS Noetic and the isolated humanoid catkin workspace; nothing is
# persisted in ~/.bashrc or any other system file.

export ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HNN_WS="${HNN_WS:-$HOME/humanoid_ws}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f "$HNN_WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HNN_WS/devel/setup.bash"
fi

# Make the project importable outside catkin as well (tools/tests).
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
export HNN_VENV="$ROOT/.venv"

# The demo video runs on the real GNOME desktop (:0); gdm keeps the X cookie
# in /run/user/1000/gdm/Xauthority on the VMware guest.
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ] || [ ! -f "${XAUTHORITY:-/nonexistent}" ]; then
  for cand in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
    if [ -f "$cand" ]; then
      export XAUTHORITY="$cand"
      break
    fi
  done
fi
