#!/usr/bin/env bash
# Desktop demonstration script shown in the recorded Ubuntu video.
# Commands are echoed with a typing effect, then really executed: the real
# ROS Noetic stack, the real Hill-muscle arm dynamics and the real
# muscle-synergy reaching missions in the reach / shelf / draw tasks.
#
#   bash scripts/demo_desktop.sh [scene] [teleop_s] [reach_s] [shelf_s] [draw_s]
#
# The 5th argument is optional; when > 0 a third reaching task (draw, a
# five-point closed trajectory) is appended.  Example recording command:
#   bash scripts/demo_desktop.sh reach 20 18 18 40
set -o pipefail

SCENE="${1:-reach}"
TELEOP_SECONDS="${2:-20}"
REACH_SECONDS="${3:-60}"
SHELF_SECONDS="${4:-60}"
DRAW_SECONDS="${5:-0}"
# keep DRAW_SECONDS a plain non-negative integer (anything else means "skip")
case "$DRAW_SECONDS" in
  ''|*[!0-9]*) DRAW_SECONDS=0 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Reproducible muscle-activation timeline for the teleop recording
# (key:seconds, '+' = combo): co-contract, lift the arm, small shoulder/elbow
# adjustments, then relax; validated collision-free in the reach scene.
TELEOP_SCRIPT="q:0.30,a:0.10,none:1.8,e:0.10,none:1.0,q:0.08,none:1.0,s:0.5,space:0.5"

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
echo "  MSK-Arm - ROS Noetic 肌肉骨骼人（Hill 肌肉驱动手臂）仿真演示"
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
echo "---- 1) Hill 肌肉驱动双连杆手臂 + 键盘遥控（Q/E 肩, A/D 肘, W 共激活, S 放松）----"
type_cmd "roslaunch msk_arm_nn teleop.launch scene:=$SCENE script:=$TELEOP_SCRIPT"
timeout -s INT "$TELEOP_SECONDS" roslaunch msk_arm_nn teleop.launch \
  scene:="$SCENE" script:="$TELEOP_SCRIPT" 2>/dev/null
sleep 1.5

echo
echo "---- 2) 肌肉协同够取任务（reach）：深度相机 + 工作空间规划 + 肌力映射 ----"
type_cmd "roslaunch msk_arm_nn reach.launch scene:=reach"
timeout -s INT "$REACH_SECONDS" roslaunch msk_arm_nn reach.launch scene:=reach 2>/dev/null
sleep 1.5

echo
echo "---- 3) 同一套算法切换到隔板绕行任务（shelf），验证多任务泛化 ----"
type_cmd "roslaunch msk_arm_nn reach.launch scene:=shelf"
timeout -s INT "$SHELF_SECONDS" roslaunch msk_arm_nn reach.launch scene:=shelf 2>/dev/null
sleep 1.5

echo
if [ "$DRAW_SECONDS" -gt 0 ]; then
  echo "---- 4) 轨迹绘制任务（draw）：连续到达 5 个目标点，验证连续够取与肌肉协同 ----"
  type_cmd "roslaunch msk_arm_nn reach.launch scene:=draw"
  timeout -s INT "$DRAW_SECONDS" roslaunch msk_arm_nn reach.launch scene:=draw 2>/dev/null
  sleep 1.5
fi

echo
echo "---- 5) 运行结果（ROS 节点写出的 JSON 指标）----"
type_cmd "ls -lh results/ | tail -n 10"
ls -lh results/ | tail -n 10
echo
echo "DEMO_FINISHED - 关闭终端窗口"
sleep 2
