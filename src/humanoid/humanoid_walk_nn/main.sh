#!/usr/bin/env bash
# =============================================================================
# HNN 一键入口：简化双足步行仿真 / 激光 SLAM / 神经网络导航
#
#   bash main.sh teleop      [hall|warehouse|obstacle]   键盘遥控（步行 + 平衡 + 避障）
#   bash main.sh slam_nav    [hall|warehouse|obstacle]   激光建图 + A* 自主步行导航
#   bash main.sh perception  [hall|warehouse|obstacle]   感知网络激光去噪
#   bash main.sh e2e         [hall|warehouse|obstacle]   端到端 CNN 深度图像导航
#   bash main.sh benchmark                               四控制器多场景闭环评测（写 results/benchmark.json/.csv）
#   bash main.sh benchmark_ros [hall|warehouse|obstacle] 单场景 ROS 闭环（metrics_node 写 results/ros_*.json）
#   bash main.sh test                                    运行物理/建图/规划/控制自测
#   bash main.sh train                                   重新训练全部神经网络
#   bash main.sh setup                                   创建项目私有 Python 环境
# =============================================================================
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${HNN_WS:-$HOME/humanoid_ws}"
MODE="${1:-teleop}"
SCENE="${2:-hall}"
PY="$ROOT/.venv/bin/python"
SCENES="hall,warehouse,obstacle"

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
  benchmark)
    if [ ! -x "$PY" ]; then
      echo "[main] 缺少 .venv，请先执行: bash main.sh setup" >&2
      exit 1
    fi
    # 四控制器 x 三场景、每格 3 条 episode、同一 seed、60 s 上限（文档 9.1 的协议）
    "$PY" "$ROOT/tools/evaluate.py" --scenes "$SCENES" \
      --episodes 3 --max-seconds 60
    exit $?
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
    "$PY" "$ROOT/tools/evaluate.py" --scenes "$SCENES" \
      --episodes 3 --max-seconds 60
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
  teleop|slam_nav|perception|e2e)
    exec roslaunch humanoid_walk_nn "${MODE}.launch" scene:="$SCENE"
    ;;
  benchmark_ros)
    exec roslaunch humanoid_walk_nn "benchmark.launch" scene:="$SCENE"
    ;;
  *)
    echo "未知模式: $MODE" >&2
    echo "用法: bash main.sh [teleop|slam_nav|perception|e2e|benchmark|benchmark_ros|test|train|setup] [hall|warehouse|obstacle]" >&2
    exit 2
    ;;
esac
