#!/bin/bash
# CARLA 端到端神经网络（图像 → 控制）—— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                                  # 在线：端到端 CNN 自主驾驶
#   bash main.sh --mode collect --host <IP>       # 行为克隆采集 (图像, 专家转向)
#   bash main.sh --mode train --epochs 60         # 训练 CNN（无需 CARLA）
#   bash main.sh --headless --demo --save_dir ~/shots
#                                                 # 离线取证：合成道路图像训练并导出图
#   bash main.sh --launch                         # 由 ROS 2 launch 启动
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

for a in "$@"; do
    if [ "$a" = "--launch" ]; then
        exec ros2 launch carla_end_to_end_nn main.launch.py "${@/--launch/}"
    fi
done

echo "=================================================="
echo "  CARLA 端到端神经网络（图像 → 控制）"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
