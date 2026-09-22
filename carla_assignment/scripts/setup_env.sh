#!/usr/bin/env bash
# CARLA 0.9.16 无人车四次作业 · 环境一键安装（Ubuntu 20.04 Noetic / 22.04 Humble）
#
# 用法： bash scripts/setup_env.sh
#
# 需要：
#   * 已安装 CARLA 0.9.16（或更高 0.9.x，接口向后兼容）。
#   * 显卡：建议 NVIDIA（CARLA 3D 对 GPU 有要求）。无 GPU 虚拟机跑 CARLA 会极卡，
#     建议改用同一目录的 MuJoCo 方案（mujoco_assignment）。
#   * (可选) ROS1 Noetic 或 ROS2 Humble，用于 roslaunch/ros2 launch。

set -e

echo "==== CARLA 无人车作业 · 环境准备 ===="

# 1) 系统包（python3 + 编译/ROS 基础）
sudo apt update -qq || true
sudo apt install -y python3 python3-pip python3-venv libjpeg-dev zlib1g-dev \
    python3-opencv || true

# 2) 创建 venv（Ubuntu 20.04 默认 python3.8 即可，carla python api 不要求 3.9+）
if [ ! -d .venv ]; then
    python3 -m venv .venv
    echo "已创建 .venv"
fi
source .venv/bin/activate
python -m pip install --upgrade pip

# 3) Python 依赖
python -m pip install numpy opencv-python-headless pygame tensorflow-cpu

# 4) CARLA Python API
#    从 CARLA 0.9.16 安装目录复制对应版本 egg，或用 pip 直装：
#    pip install carla==0.9.16   (或从 ~/carla/PythonAPI/carla/dist/*.whl)
PYPATH=$(python -c "import sys; v=sys.version_info; print(f'cp{v.major}{v.minor}-cp{v.major}{v.minor}')" 2>/dev/null || echo "")
echo "== 尝试安装 carla Python API ($PYPATH) =="
python -m pip install "carla==0.9.16" 2>/dev/null \
    || python -m pip install carla \
    || echo "[提示] 无法自动装 carla，请从 CARLA 的 PythonAPI 手动安装："
    || echo "       pip install /path/to/Carla/PythonAPI/carla/dist/*.whl"

# 5) 验证
echo
echo "==== 验证 ===="
python -c "import carla; print('carla API OK:', carla.__file__)" 2>&1 || echo "[警告] carla 模块未装"
python -c "import numpy; print('numpy', numpy.__version__)"
python -c "import cv2; print('opencv', cv2.__version__)"
python -c "import pygame; print('pygame', pygame.ver)" 2>/dev/null || echo "[警告] pygame 未装"
echo
echo "环境就绪。以后每次：source .venv/bin/activate"
