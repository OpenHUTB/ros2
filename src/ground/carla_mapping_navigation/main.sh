#!/bin/bash
# CARLA 建图 + 导航（神经网络规划）—— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                       # 在线：LiDAR 建图 + NN 规划导航
#   bash main.sh --mode train          # 离线训练规划神经网络（无需 CARLA）
#   bash main.sh --host <IP>           # 指定 CARLA 服务端（虚拟机填宿主机 IP）
#   bash main.sh --headless --demo --save_dir ~/shots
#                                      # 离线取证：合成环境建图+导航并导出地图（无 CARLA 也能跑）
#   bash main.sh --launch <参数>:=<值> ...   # 交给 ROS launch 启动
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

# --launch：交给 ROS launch 启动。ROS 2 走 ros2 launch，ROS 1 走 roslaunch，
# 其余参数按 launch 的写法原样传递。注意只能用两个 launch 都声明过的参数
# （host / port / town / goal / goal_x / goal_y），否则换一个 ROS 版本就不生效：
#   bash main.sh --launch host:=192.168.8.1
#   bash main.sh --launch host:=192.168.8.1 goal:="20,8"
launch_args=()
want_launch=0
for a in "$@"; do
    if [ "$a" = "--launch" ]; then
        want_launch=1
    else
        launch_args+=("$a")
    fi
done

if [ "$want_launch" = "1" ]; then
    if [ "${ROS_VERSION:-}" = "2" ] || [ -d /opt/ros/humble ]; then
        exec ros2 launch carla_mapping_navigation main.launch.py "${launch_args[@]}"
    elif [ "${ROS_VERSION:-}" = "1" ] || [ -d /opt/ros/noetic ]; then
        exec roslaunch carla_mapping_navigation main.launch "${launch_args[@]}"
    fi
    echo "未检测到 ROS 环境：请先 source /opt/ros/humble/setup.bash" >&2
    echo "或 source /opt/ros/noetic/setup.bash 后再用 --launch。" >&2
    exit 1
fi

echo "=================================================="
echo "  CARLA 建图 + 导航（神经网络规划）"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
