#!/usr/bin/env bash
# Desktop demonstration script shown in the recorded Ubuntu video.
# Commands are echoed with a typing effect, then really executed: the real
# ROS Noetic stack, the real bicycle-model road vehicle and the real SLAM/A*
# mission in the downtown and construction road networks.
set -o pipefail

SCENE="${1:-downtown}"
TELEOP_SECONDS="${2:-26}"
NAV_SECONDS="${3:-90}"
CONSTRUCTION_SECONDS="${4:-70}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Reproducible keyboard timeline for the teleop recording (key:seconds,
# '+' = combo), validated by the lead in downtown: 0 collisions, 0 red-light
# violations, 0 pedestrians hit, stays on the road (12 s incl. a red-light
# stop); the teleop segment runs 26 s so the car idles after the timeline.
TELEOP_SCRIPT="w:2.6,space:2.0,w:2.2,s:1.6,space:0.8,w:1.8,space:1.0"

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
echo "  GRNN - ROS Noetic 地面载具（城市道路自动驾驶车）仿真演示"
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
echo "---- 1) 自行车模型车辆物理仿真 + 键盘遥控（W/S 油门刹车, A/D 转向）----"
type_cmd "roslaunch road_numpy_nn_slam teleop.launch scene:=$SCENE script:=$TELEOP_SCRIPT"
timeout -s INT "$TELEOP_SECONDS" roslaunch road_numpy_nn_slam teleop.launch \
  scene:="$SCENE" script:="$TELEOP_SCRIPT" 2>/dev/null
sleep 1.5

echo
echo "---- 2) 激光建图 + 扫描匹配定位 + A* 规划 + 红绿灯/行人自主导航（downtown）----"
type_cmd "roslaunch road_numpy_nn_slam slam_nav.launch scene:=downtown"
timeout -s INT "$NAV_SECONDS" roslaunch road_numpy_nn_slam slam_nav.launch scene:=downtown 2>/dev/null
sleep 1.5

echo
echo "---- 3) 同一套算法切换到施工路段（construction），验证多场景泛化 ----"
type_cmd "roslaunch road_numpy_nn_slam slam_nav.launch scene:=construction"
timeout -s INT "$CONSTRUCTION_SECONDS" roslaunch road_numpy_nn_slam slam_nav.launch scene:=construction 2>/dev/null
sleep 1.5

echo
echo "---- 4) 运行结果（ROS 节点写出的 JSON 指标）----"
type_cmd "ls -lh results/ | tail -n 10"
ls -lh results/ | tail -n 10
echo
echo "DEMO_FINISHED - 关闭终端窗口"
sleep 2
