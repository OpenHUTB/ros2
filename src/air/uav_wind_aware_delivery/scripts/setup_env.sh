#!/usr/bin/env bash
# 创建项目内 Python 3.11 venv（可选；已有可用的 .venv 可跳过）
set -e
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PYTHON311:-python3.11}"
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "找不到 $PY；请设置 PYTHON311=/path/to/python3.11" >&2
  exit 1
fi
if [ ! -d "$PKG/.venv" ]; then
  "$PY" -m venv "$PKG/.venv"
fi
"$PKG/.venv/bin/pip" install --upgrade pip
"$PKG/.venv/bin/pip" install -r "$PKG/requirements.txt"
# 让 venv 能 import rospy/genpy（不修改系统 Python）
SITE="$("$PKG/.venv/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
echo /opt/ros/noetic/lib/python3/dist-packages > "$SITE/ros_noetic.pth"
echo "venv ready: $PKG/.venv"
