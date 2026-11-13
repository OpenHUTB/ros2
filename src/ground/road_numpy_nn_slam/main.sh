#!/usr/bin/env bash
# =============================================================================
# GRNN-Road 一键入口：2-D 自行车动力学仿真 / 激光 SLAM / 神经网络导航
#
#   bash main.sh teleop      [downtown|campus|construction]  键盘遥控（物理仿真 + 红绿灯/行人）
#   bash main.sh slam_nav    [downtown|campus|construction]  激光建图 + A* 自主导航
#   bash main.sh perception  [downtown|campus|construction]  感知网络激光去噪
#   bash main.sh e2e         [downtown|campus|construction]  端到端 CNN 图像导航
#   bash main.sh benchmark   [downtown|campus|construction]  指标评测
#   bash main.sh test                                        运行物理/建图/规划/控制自测
#   bash main.sh train                                       重新训练全部神经网络
#   bash main.sh setup                                       创建项目私有 Python 环境
# =============================================================================
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${GRNN_WS:-$HOME/road_ws}"
MODE="${1:-teleop}"
SCENE="${2:-downtown}"
PY="$ROOT/.venv/bin/python"
SCENES="downtown,campus,construction"

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
    "$PY" "$ROOT/tools/train_perception.py"
    "$PY" "$ROOT/tools/train_control.py"
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
  teleop|slam_nav|perception|e2e|benchmark)
    exec roslaunch road_numpy_nn_slam "${MODE}.launch" scene:="$SCENE"
    ;;
  *)
    echo "未知模式: $MODE" >&2
    echo "用法: bash main.sh [teleop|slam_nav|perception|e2e|benchmark|test|train|setup] [downtown|campus|construction]" >&2
    exit 2
    ;;
esac
