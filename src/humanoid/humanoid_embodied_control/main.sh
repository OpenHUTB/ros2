#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# humanoid_embodied_control —— 课程约定一键运行入口（main.sh）
#
# 用法：
#   bash main.sh --mode sim --seconds 30
#   bash main.sh --mode offline
#   bash main.sh --help
#
# 说明：ROS 1 Noetic 自带 python3.8，而 MuJoCo 3.x 需要 3.9+。
# 因此本脚本按以下优先级挑选解释器：
#   1. 环境变量显式指定的 HUMANoid_PYTHON
#   2. python3.10 / python3.11（若存在）
#   3. 系统 python3
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

pick_python() {
  if [ -n "${HUMANOID_PYTHON:-}" ]; then
    echo "$HUMANOID_PYTHON"
    return
  fi
  for cand in python3.11 python3.10 python3; do
    if command -v "$cand" >/dev/null 2>&1; then
      ver="$("$cand" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo 0.0)"
      if [ "$(printf '%s\n3.9\n' "$ver" | sort -V | head -n1)" = "3.9" ]; then
        echo "$cand"
        return
      fi
    fi
  done
  echo "python3"
}

PY="$(pick_python)"
echo "[humanoid_embodied_control] using $PY ($("$PY" --version 2>&1))"

# MuJoCo 不存在时自动安装（离线镜像可用 --no-mujoco 跳过，只跑 --mode offline）
if ! "$PY" -c "import mujoco" >/dev/null 2>&1; then
  echo "[humanoid_embodied_control] mujoco 未安装，尝试安装..."
  "$PY" -m pip install -q mujoco numpy scipy || \
    echo "[humanoid_embodied_control] 安装失败，仍可运行 --mode offline"
fi

exec "$PY" main.py "$@"
