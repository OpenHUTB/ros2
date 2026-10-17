#!/bin/bash
# CARLA 端到端神经网络（图像 → 控制）—— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                                  # 在线：端到端 CNN 自主驾驶
#   bash main.sh --mode collect --host <IP>       # 行为克隆采集 (图像, 专家转向)
#   bash main.sh --mode train --epochs 60         # 训练 CNN（无需 CARLA）
#   bash main.sh --headless --demo --save_dir ~/shots
#                                                 # 离线取证：合成道路图像训练并导出图
#   bash main.sh --launch <参数>:=<值> ...       # 交给 ROS launch 启动
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
# 其余参数按 launch 的写法原样传递，例如：
#   bash main.sh --launch host:=192.168.8.1
#
# 注意两个 launch 接受的参数**不完全一样**，传错了不会报错而是被静默忽略：
#   ROS 2 main.launch.py : host, port, town, model_path, backend, config
#   ROS 1 main.launch    : 上面这些（无 config）+ mode, sim_time
# 例如 mode:= 只有 ROS 1 认；在 Humble 上传 mode:=test 不会报错，但也不起作用。
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
        exec ros2 launch carla_end_to_end_nn main.launch.py "${launch_args[@]}"
    elif [ "${ROS_VERSION:-}" = "1" ] || [ -d /opt/ros/noetic ]; then
        exec roslaunch carla_end_to_end_nn main.launch "${launch_args[@]}"
    fi
    echo "未检测到 ROS 环境：请先 source /opt/ros/humble/setup.bash" >&2
    echo "或 source /opt/ros/noetic/setup.bash 后再用 --launch。" >&2
    exit 1
fi

echo "=================================================="
echo "  CARLA 端到端神经网络（图像 → 控制）"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
