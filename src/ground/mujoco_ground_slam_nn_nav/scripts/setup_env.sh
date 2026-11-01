#!/usr/bin/env bash
# Create the project-local Python 3.11 environment.  No shell profile, system
# Python, environment variable or ROS installation is modified.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"
ROS_PY="/opt/ros/noetic/lib/python3/dist-packages"

if [ -x "$VENV/bin/python3" ] && "$VENV/bin/python3" -c "import mujoco" 2>/dev/null; then
  echo "[setup] existing venv is usable: $VENV"
  if [ -d "$ROS_PY" ] && [ ! -f "$VENV/lib/python3.11/site-packages/ros_noetic.pth" ]; then
    echo "$ROS_PY" > "$VENV/lib/python3.11/site-packages/ros_noetic.pth"
  fi
  exit 0
fi

PY="${PYTHON311:-}"
if [ -z "$PY" ]; then
  if command -v python3.11 >/dev/null 2>&1; then
    PY="$(command -v python3.11)"
  elif [ -x "$ROOT/.python/bin/python3.11" ]; then
    PY="$ROOT/.python/bin/python3.11"
  else
    cat <<'MSG'
[setup] Python >= 3.11 was not found.
        MuJoCo 3.12.0 requires Python >= 3.10, while Ubuntu 20.04 ships 3.8.
        Install Python 3.11 (deadsnakes, pyenv or conda), then re-run:
            PYTHON311=/path/to/python3.11 bash scripts/setup_env.sh
        Alternatively, activate any Python 3.11 environment and run:
            python -m pip install -r requirements.txt
MSG
    exit 1
  fi
fi

echo "[setup] using $PY"
"$PY" -m venv "$VENV"
"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r "$ROOT/requirements.txt"
if [ -d "$ROS_PY" ]; then
  echo "$ROS_PY" > "$VENV/lib/python3.11/site-packages/ros_noetic.pth"
fi
echo "[setup] done. The venv is private to this repository."
