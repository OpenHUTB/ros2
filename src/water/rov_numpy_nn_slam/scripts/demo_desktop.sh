#!/usr/bin/env bash
# Desktop demonstration script shown in the recorded Ubuntu video.
# Commands are echoed with a typing effect, then really executed: the real
# ROS Noetic stack, the real MuJoCo physics and the real SLAM/A* mission.
set -o pipefail

SCENE="${1:-pool}"
TELEOP_SECONDS="${2:-26}"
NAV_SECONDS="${3:-40}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Same keyboard timeline as tools/record_gif.py (key:seconds, '+' = combo).
TELEOP_SCRIPT="w:1.8,a:0.9,w:1.8,a:0.9,w:1.8,a:0.9,w:1.8,a:0.9,r:1.2,f:1.2,q:1.0,e:1.0,a:0.8,w:1.2,d:0.8,s:1.2,w:1.2,a:0.9,w:1.0"

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
echo "  RVNN - ROS Noetic + MuJoCo 水下机器人仿真与神经导航演示"
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
echo "---- 1) MuJoCo 物理仿真 + 键盘遥控（W/A/S/D 平移转向, R/F 升沉, Q/E 横移）----"
type_cmd "roslaunch rov_numpy_nn_slam teleop.launch scene:=$SCENE script:=$TELEOP_SCRIPT"
timeout -s INT "$TELEOP_SECONDS" roslaunch rov_numpy_nn_slam teleop.launch \
  scene:="$SCENE" script:="$TELEOP_SCRIPT" 2>/dev/null
sleep 1.5

echo
echo "---- 2) 声呐建图 + 扫描匹配定位 + A* 规划 + 自主导航（泳池 pool）----"
type_cmd "roslaunch rov_numpy_nn_slam slam_nav.launch scene:=pool"
timeout -s INT "$NAV_SECONDS" roslaunch rov_numpy_nn_slam slam_nav.launch scene:=pool 2>/dev/null
sleep 1.5

echo
echo "---- 3) 同一套算法切换到水下洞穴场景（cave），验证多场景泛化 ----"
type_cmd "roslaunch rov_numpy_nn_slam slam_nav.launch scene:=cave"
timeout -s INT "$NAV_SECONDS" roslaunch rov_numpy_nn_slam slam_nav.launch scene:=cave 2>/dev/null
sleep 1.5

echo
echo "---- 4) 运行结果（ROS 节点写出的 JSON 指标）----"
type_cmd "ls -lh results/ | tail -n 10"
ls -lh results/ | tail -n 10
echo
echo "DEMO_FINISHED - 关闭终端窗口"
sleep 2
