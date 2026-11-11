#!/usr/bin/env bash
# mujoco_car_nn_avoidance 一键入口（与 main.py 等价，便于在虚拟机/WSL 里直接运行）
#
#   bash main.sh --eval          # 无参数时默认跑闭环评测并导出结果
#   bash main.sh --collect       # 采集专家标签数据
#   bash main.sh --train         # 训练 MLP
#   bash main.sh --demo         # 录制运行 gif
#   bash main.sh --test          # 自检
#   bash main.sh --launch        # ROS 2 一键启动
#
# 注意：提交前确认本文件在 git 中带可执行位（100755），
# roslaunch / 部分评审环境依赖 shebang 直接执行：
#   chmod +x main.sh
#   git update-index --chmod=+x main.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -eq 0 ]; then
    exec python3 "${SCRIPT_DIR}/main.py" --eval
fi

exec python3 "${SCRIPT_DIR}/main.py" "$@"
