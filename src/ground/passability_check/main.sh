#!/usr/bin/env bash
# passability_check 一键入口（与 main.py 等价，便于在虚拟机 / WSL 里直接运行）
#
#   bash main.sh                          跑默认车型 + 全部场景
#   bash main.sh --vehicle suv            换车型
#   bash main.sh --vehicle all --matrix   看车型 × 场景对比表
#   bash main.sh --test                   自检（不依赖 ROS）
#   bash main.sh --launch                 ROS 2 一键启动
#
# 提交前确认本文件在 git 中带可执行位（100755）：
#   git update-index --chmod=+x main.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -eq 0 ]; then
    exec python3 "${SCRIPT_DIR}/main.py"
fi

exec python3 "${SCRIPT_DIR}/main.py" "$@"
