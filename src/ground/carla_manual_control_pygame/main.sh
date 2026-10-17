#!/bin/bash
# carla_manual_control 启动入口

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 激活 ROS Noetic
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 激活 conda 环境（carla38 含 hutb、pygame）
if command -v conda &> /dev/null; then
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate carla38
fi

# Carla 客户端依赖，避免 undefined symbol 错误
export LD_PRELOAD=/home/user/miniconda3/envs/carla38/lib/libc++.so.1:/home/user/miniconda3/envs/carla38/lib/libc++abi.so.1

cd "$SCRIPT_DIR"
exec python main.py "$@"
