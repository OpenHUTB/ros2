#!/bin/bash
# CARLA 建图 + 导航（神经网络规划）—— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                       # 在线：LiDAR 建图 + NN 规划导航
#   bash main.sh --mode train          # 离线训练规划神经网络（无需 CARLA）
#   bash main.sh --host <IP>           # 指定 CARLA 服务端（虚拟机填宿主机 IP）
#   bash main.sh --headless --demo --save_dir ~/shots
#                                      # 离线取证：合成环境建图+导航并导出地图（无 CARLA 也能跑）
#   bash main.sh --launch              # 由 ROS 2 launch 启动
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
        exec ros2 launch carla_mapping_navigation main.launch.py "${@/--launch/}"
    fi
done

echo "=================================================="
echo "  CARLA 建图 + 导航（神经网络规划）"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
