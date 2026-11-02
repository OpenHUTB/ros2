#!/usr/bin/env bash
# 人形机器人全身逆运动学与可操作性分析 —— 启动脚本
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE/scripts"

PY="${PYTHON:-python3}"
"$PY" main.py "$@"
