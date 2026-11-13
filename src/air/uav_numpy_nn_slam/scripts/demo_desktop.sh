#!/usr/bin/env bash
# Desktop demonstration script shown in the recorded Ubuntu video.
# Commands are echoed with a typing effect, then really executed: the real
# ROS Noetic stack, the real MuJoCo quadrotor physics and the real SLAM/A*
# aerial mission in the city and canyon airspaces.
set -o pipefail

SCENE="${1:-city}"
TELEOP_SECONDS="${2:-26}"
NAV_SECONDS="${3:-65}"
NAV2_SECONDS="${4:-40}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Same keyboard timeline as tools/record_gif.py and teleop.launch
# (key:seconds, '+' = combo), validated collision-free in the city airspace.
TELEOP_SCRIPT="r:2.5,a:1.5,f:2.0,w:1.2,d:1.5,q:1.0,e:1.0,r:1.5,w:1.2,a:1.5,s:1.2,f:1.5,d:1.5,r:1.0,q:1.0,w:1.2,a:1.5,s:1.2,f:1.5,d:1.5,e:1.2"

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
echo "  UVNN - ROS Noetic + MuJoCo 无人机（空域载具）仿真与神经导航演示"
echo "  虚拟机: Ubuntu 20.04.6 LTS   ROS: Noetic   场景: $SCENE"
echo "=============================================================="
sleep 1.0

type_cmd "source /opt/ros/noetic/setup.bash"
source /opt/ros/noetic/setup.bash
sleep 0.25
type_cmd "source $ROOT/scripts/env.sh"
# shellcheck disable=SC1091
source "$ROOT/scripts/env.sh"
sleep 0.4

echo
echo "---- 1) MuJoCo 四旋翼物理仿真 + 键盘遥控（W/A/S/D 前后转向, R/F 升降, Q/E 横移）----"
type_cmd "roslaunch uav_numpy_nn_slam teleop.launch scene:=$SCENE script:=$TELEOP_SCRIPT"
timeout -s INT "$TELEOP_SECONDS" roslaunch uav_numpy_nn_slam teleop.launch \
  scene:="$SCENE" script:="$TELEOP_SCRIPT" 2>/dev/null
sleep 1.5

echo
echo "---- 2) 激光建图 + 扫描匹配定位 + A* 规划 + 自主导航（城市空域 city）----"
type_cmd "roslaunch uav_numpy_nn_slam slam_nav.launch scene:=city"
timeout -s INT "$NAV_SECONDS" roslaunch uav_numpy_nn_slam slam_nav.launch scene:=city 2>/dev/null
sleep 1.5

echo
echo "---- 3) 同一套算法切换到峡谷空域（canyon），验证多场景泛化 ----"
type_cmd "roslaunch uav_numpy_nn_slam slam_nav.launch scene:=canyon"
timeout -s INT "$NAV2_SECONDS" roslaunch uav_numpy_nn_slam slam_nav.launch scene:=canyon 2>/dev/null
sleep 1.5

echo
echo "---- 4) 运行结果（ROS 节点写出的 JSON 指标）----"
type_cmd "ls -lh results/ | tail -n 10"
ls -lh results/ | tail -n 10
echo
echo "DEMO_FINISHED - 关闭终端窗口"
sleep 2
