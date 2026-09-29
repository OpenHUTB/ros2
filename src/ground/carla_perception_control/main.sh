#!/bin/bash
# CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）—— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                          # 在线：连 CARLA，NN 感知 + 轨迹跟踪
#   bash main.sh --mode train             # 离线训练两个神经网络（无需 CARLA）
#   bash main.sh --host <IP>              # 指定 CARLA 服务端（虚拟机填宿主机 IP）
#   bash main.sh --headless --demo --save_dir ~/shots
#                                         # 离线取证：训练+回放并导出曲线图（无 CARLA 也能跑）
#   bash main.sh --host 192.168.8.1 --follow
#                                         # 第三人称跟随镜头（观察/录屏用）
#   bash main.sh --launch                 # 由 ROS 2 launch 启动
#
# 连不上 CARLA 时先做连通性诊断（分步打印耗时，指出卡在哪一环）：
#   python3 check_connection.py <宿主机IP> 2000 Town05
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# 加载 ROS 2 Humble 环境（若存在），供 --launch 模式使用
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

# --launch：交给 ros2 launch
for a in "$@"; do
    if [ "$a" = "--launch" ]; then
        exec ros2 launch carla_perception_control main.launch.py "${@/--launch/}"
    fi
done

echo "=================================================="
echo "  CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）"
echo "=================================================="

# 优先使用 python3.10（CARLA 0.9.16 客户端 wheel 的 ABI 版本），否则退回 python3
PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
