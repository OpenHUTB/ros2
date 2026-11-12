#!/usr/bin/env bash
# =============================================================================
# MSK-Arm 一键入口：Hill 肌肉驱动手臂仿真 / 工作空间建图 / 神经网络控制
#
#   bash main.sh teleop      [reach|shelf|draw]   键盘肌肉激活（物理仿真 + 肌肉激活可视化）
#   bash main.sh reach       [reach|shelf|draw]   深度建图 + A* + 肌肉协同够取
#   bash main.sh perception  [reach|shelf|draw]   感知网络激光/深度去噪
#   bash main.sh e2e         [reach|shelf|draw]   端到端 CNN 深度图 -> 肌肉激活
#   bash main.sh benchmark   [reach|shelf|draw]   指标评测
#   bash main.sh test                             运行物理/建图/规划/控制自测
#   bash main.sh train                            重新训练全部神经网络
#   bash main.sh setup                            创建项目私有 Python 环境
# =============================================================================
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${MSK_WS:-$HOME/msk_ws}"
MODE="${1:-teleop}"
SCENE="${2:-reach}"
PY="$ROOT/.venv/bin/python"
SCENES="reach,shelf,draw"

if [ ! -d /opt/ros/noetic ]; then
  echo "[main] 未找到 ROS Noetic（/opt/ros/noetic），请先在 Ubuntu 20.04 安装 ROS Noetic。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

case "$MODE" in
  setup)
    bash "$ROOT/scripts/setup_env.sh"
    exit $?
    ;;
  test)
    if [ ! -x "$PY" ]; then
      echo "[main] 缺少 .venv，请先执行: bash main.sh setup" >&2
      exit 1
    fi
    rc=0
    for t in "$ROOT"/tests/smoke_sim.py "$ROOT"/tests/test_*.py; do
      echo "=== $(basename "$t") ==="
      "$PY" "$t" || rc=1
    done
    exit $rc
    ;;
  train)
    if [ ! -x "$PY" ]; then
      echo "[main] 缺少 .venv，请先执行: bash main.sh setup" >&2
      exit 1
    fi
    "$PY" "$ROOT/tools/collect_expert.py" --scenes "$SCENES"
    "$PY" "$ROOT/tools/train_activation.py"
    "$PY" "$ROOT/tools/train_ik.py"
    "$PY" "$ROOT/tools/train_planner.py"
    "$PY" "$ROOT/tools/train_e2e.py"
    "$PY" "$ROOT/tools/train_rl.py"
    "$PY" "$ROOT/tools/evaluate.py" --scenes "$SCENES"
    exit $?
    ;;
esac

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "[main] 首次运行，正在编译独立 catkin 工作空间: $WS"
  bash "$ROOT/scripts/build_ros.sh" "$WS" || exit 1
fi
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

case "$MODE" in
  teleop|reach|perception|e2e|benchmark)
    exec roslaunch msk_arm_nn "${MODE}.launch" scene:="$SCENE"
    ;;
  *)
    echo "未知模式: $MODE" >&2
    echo "用法: bash main.sh [teleop|reach|perception|e2e|benchmark|test|train|setup] [reach|shelf|draw]" >&2
    exit 2
    ;;
esac
