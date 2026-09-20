#!/usr/bin/env bash
# 双海龟"追逐—逃亡"协同控制实验 —— 一键运行脚本 main.sh
#
# 用法: bash main.sh    (在本模块目录 src/chap1/turtle_chase/ 下执行)
# 功能: 自动拷贝功能包到 catkin 工作空间、编译并 source, 检测并启动
#       roscore 与 turtlesim 仿真器, 再拉起逃亡者 runner.py 和追逐者
#       chaser.py; 按 Ctrl+C 退出时自动清理全部子进程。

set -u

MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/catkin_ws"

# 0. ROS 环境未加载时自动 source
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/*/setup.bash
fi

# 1. 拷贝功能包到工作空间(从工作空间内部运行时跳过, 避免目录自拷贝)
if [ "$MODULE_DIR" != "$WS_DIR/src/turtle_chase" ]; then
    mkdir -p "$WS_DIR/src"
    echo "[main.sh] 拷贝功能包到 $WS_DIR/src/"
    cp -r "$MODULE_DIR" "$WS_DIR/src/"
fi

# 2. 编译并 source
echo "[main.sh] 编译工作空间..."
(cd "$WS_DIR" && catkin_make) || { echo "[main.sh] catkin_make 失败, 请检查报错"; exit 1; }
source "$WS_DIR/devel/setup.bash"

# 3. 检测 roscore, 没有就启动一个
if ! rostopic list >/dev/null 2>&1; then
    echo "[main.sh] 启动 roscore..."
    roscore &
    sleep 3
fi

# 4. 启动仿真器与两个节点
rosrun turtlesim turtlesim_node &
TURTLE_PID=$!
sleep 2
rosrun turtle_chase runner.py &
RUNNER_PID=$!
sleep 1
rosrun turtle_chase chaser.py &
CHASER_PID=$!

cleanup() {
    echo "[main.sh] 正在退出, 清理全部子进程..."
    kill $CHASER_PID $RUNNER_PID $TURTLE_PID 2>/dev/null
    wait 2>/dev/null
    echo "[main.sh] 已退出"
}
trap cleanup EXIT INT TERM

echo "[main.sh] 实验运行中, 按 Ctrl+C 一键退出"
wait
