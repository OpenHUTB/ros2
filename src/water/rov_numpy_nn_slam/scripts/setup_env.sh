#!/usr/bin/env bash
# Create the project-private Python environment used for MuJoCo physics.
# Nothing outside this repository is modified: no system Python package, no
# ~/.bashrc, no global environment variable.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"
ROS_PY="/opt/ros/noetic/lib/python3/dist-packages"

if [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -c "import mujoco" 2>/dev/null; then
  echo "[setup] venv already usable: $VENV"
else
  PY="${PYTHON311:-}"
  if [ -z "$PY" ]; then
    for cand in python3.11 python3.10; do
      if command -v "$cand" >/dev/null 2>&1; then PY="$(command -v "$cand")"; break; fi
    done
  fi
  if [ -z "$PY" ]; then
    echo "[setup] Python >= 3.10 was not found (Ubuntu 20.04 ships 3.8)." >&2
    echo "[setup] Install python3.11 or set PYTHON311=/path/to/python3.11." >&2
    exit 1
  fi
  echo "[setup] using $PY"
  "$PY" -m venv --system-site-packages "$VENV"
  "$VENV/bin/python" -m pip install --upgrade pip
  "$VENV/bin/python" -m pip install -r "$ROOT/requirements.txt"
fi

# Make the ROS Noetic Python packages visible inside the venv so that rospy
# nodes can run with the MuJoCo-capable interpreter.
mkdir -p "$VENV/lib/python"*"/site-packages"
printf '%s\n' "$ROS_PY" > "$VENV/lib/python"*"/site-packages/ros_noetic.pth"
echo "[setup] done -> $VENV"
echo "[setup] Verify: $VENV/bin/python -c 'import mujoco, rospy; print(mujoco.__version__)'"
