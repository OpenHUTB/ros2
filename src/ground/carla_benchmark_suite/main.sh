#!/bin/bash
# CARLA 作业综合整合与性能评价 —— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                              # 打印帮助
#   bash main.sh --list                       # 列出所有可调度模块
#   bash main.sh --benchmark --save_dir ~/shots --out report.json
#                                             # 运行基准评测（真实测量，无需 CARLA）
#   bash main.sh --target perception -- --headless --demo --save_dir ~/shots
#                                             # 调度作业二（"--" 之后参数原样透传）
#   bash main.sh --target end_to_end -- --mode train --epochs 60
#   bash main.sh --launch [name:=value ...]   # 交给 ROS launch 启动
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
#   bash main.sh --launch mode:=test host:=192.168.8.1
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
        exec ros2 launch carla_benchmark_suite main.launch.py "${launch_args[@]}"
    elif [ "${ROS_VERSION:-}" = "1" ] || [ -d /opt/ros/noetic ]; then
        exec roslaunch carla_benchmark_suite main.launch "${launch_args[@]}"
    fi
    echo "未检测到 ROS 环境：请先 source /opt/ros/humble/setup.bash" >&2
    echo "或 source /opt/ros/noetic/setup.bash 后再用 --launch。" >&2
    exit 1
fi

echo "=================================================="
echo "  CARLA 作业综合整合与性能评价"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
