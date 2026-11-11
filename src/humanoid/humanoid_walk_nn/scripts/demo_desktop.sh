#!/usr/bin/env bash
# Desktop demonstration script shown in the recorded Ubuntu video.
# Commands are echoed with a typing effect, then really executed: the real
# ROS Noetic stack, the real bipedal walking model and the real SLAM/A*
# mission in the hall and warehouse indoor scenes.
set -o pipefail

SCENE="${1:-hall}"
TELEOP_SECONDS="${2:-26}"
NAV_SECONDS="${3:-60}"
WAREHOUSE_SECONDS="${4:-60}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Reproducible keyboard timeline for the teleop recording (key:seconds,
# '+' = combo): walk straight, stop, small turns while staying in the open
# start area of the hall (>=1.4 m clearance from the first pillar at (7,6));
# the teleop segment runs 26 s so the robot idles after the timeline.
TELEOP_SCRIPT="w:1.0,space:0.8,a:0.5,w:0.8,d:0.5,w:1.0,space:1.0,w:0.6,space:1.0"

type_cmd() {
  local s="$1"
  printf '\033[1;32m$\033[0m '
  local i
  for ((i = 0; i < ${#s}; i++)); do
    printf '%s' "${s:$i:1}"
    sleep 0.022
  done
  printf '\n'
  sleep 0.2
}

clear
echo "=============================================================="
echo "  HNN - ROS Noetic 双足人形机器人（室内步行 + 建图导航）仿真演示"
echo "  虚拟机: Ubuntu 20.04.6 LTS   ROS: Noetic   场景: $SCENE"
echo "=============================================================="
sleep 1.0

type_cmd "source /opt/ros/noetic/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
sleep 0.25
type_cmd "source $ROOT/scripts/env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"
sleep 0.4

echo
echo "---- 1) 双足步行模型 + 键盘遥控（W/S 前进后退, A/D 转向, 平衡与摔倒监测）----"
type_cmd "roslaunch humanoid_walk_nn teleop.launch scene:=$SCENE script:=$TELEOP_SCRIPT"
timeout -s INT "$TELEOP_SECONDS" roslaunch humanoid_walk_nn teleop.launch \
  scene:="$SCENE" script:="$TELEOP_SCRIPT" 2>/dev/null
sleep 1.5

echo
echo "---- 2) 激光建图 + 扫描匹配定位 + A* 规划 + 走廊自主导航（hall）----"
type_cmd "roslaunch humanoid_walk_nn slam_nav.launch scene:=hall"
timeout -s INT "$NAV_SECONDS" roslaunch humanoid_walk_nn slam_nav.launch scene:=hall 2>/dev/null
sleep 1.5

echo
echo "---- 3) 同一套算法切换到仓储场景（warehouse，动态搬运车），验证多场景泛化 ----"
type_cmd "roslaunch humanoid_walk_nn slam_nav.launch scene:=warehouse"
timeout -s INT "$WAREHOUSE_SECONDS" roslaunch humanoid_walk_nn slam_nav.launch scene:=warehouse 2>/dev/null
sleep 1.5

echo
echo "---- 4) 运行结果（ROS 节点写出的 JSON 指标）----"
type_cmd "ls -lh results/ | tail -n 10"
ls -lh results/ | tail -n 10
echo
echo "DEMO_FINISHED - 关闭终端窗口"
sleep 2
