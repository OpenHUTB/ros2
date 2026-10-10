#!/bin/bash
# CARLA 0.9.16 地面载具物理仿真与键盘运动控制 —— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                 # 独立模式（pygame 窗口 + 键盘控制）
#   bash main.sh --launch        # ROS 2 Humble launch 模式
#   bash main.sh --host <IP>     # 指定 CARLA 服务端地址（虚拟机填宿主机 IP）
#   bash main.sh --host <IP> --headless --demo --save_dir ~/shots
#                                # 无窗口取证：自动跑控制序列并存图（无 3D 加速时用）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# 虚拟机（VMware / VirtualBox）常缺 3D 加速，导致 pygame 窗口黑屏或无法创建。
# 未显式指定时强制软件渲染，保证有图形界面时也能正常开窗。
if [ -z "${LIBGL_ALWAYS_SOFTWARE}" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# 加载 ROS 环境（若存在）：优先 Humble（ROS 2），否则 Noetic（ROS 1），
# 分别供 --launch / --ros1 模式使用
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

echo "=================================================="
echo "  CARLA 0.9.16 地面载具物理仿真与键盘运动控制"
echo "=================================================="

# 优先使用 python3.10（CARLA 0.9.16 客户端 wheel 的 ABI 版本），否则退回 python3
PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
