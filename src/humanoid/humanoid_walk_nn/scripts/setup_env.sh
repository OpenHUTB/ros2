#!/usr/bin/env bash
# Create the project-private Python environment used by the humanoid nodes.
# Nothing outside this repository is modified: no system Python package, no
# ~/.bashrc, no global environment variable.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"
ROS_PY="/opt/ros/noetic/lib/python3/dist-packages"
# 可选：用环境变量 PYTHON311 指定 >= 3.10 的解释器；否则自动探测。

if [ -x "$VENV/bin/python3" ] && "$VENV/bin/python3" -c "import numpy, PIL, cv2, yaml" 2>/dev/null; then
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
  if ! "$VENV/bin/python3" -c "import numpy, PIL, cv2, yaml" 2>/dev/null; then
    echo "[setup] installing requirements into the venv"
    "$VENV/bin/python3" -m pip install --upgrade pip
    "$VENV/bin/python3" -m pip install -r "$ROOT/requirements.txt"
  fi
fi

# Make the ROS Noetic Python packages visible inside the venv so that rospy
# nodes can run with the project interpreter.  (A glob is used because the venv
# directory is versioned, e.g. lib/python3.11/site-packages.)
for sp in "$VENV"/lib/python*/site-packages; do
  [ -d "$sp" ] || continue
  printf '%s\n' "$ROS_PY" > "$sp/ros_noetic.pth"
done
echo "[setup] done -> $VENV"
echo "[setup] Verify: $VENV/bin/python3 -c 'import numpy, rospy; print(\"ok\")'"
