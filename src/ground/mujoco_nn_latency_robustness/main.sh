#!/usr/bin/env bash
# 训练 + 实验 + 出图出 GIF（不需要 ROS，只要有 mujoco/numpy/matplotlib/pillow）
set -e
cd "$(dirname "$0")"
python3 -m pip install -q -r requirements.txt
MPLBACKEND=Agg python3 main.py --demo "$@"
