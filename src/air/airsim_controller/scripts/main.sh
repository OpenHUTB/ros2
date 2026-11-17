#!/usr/bin/env bash
# 统一入口（Linux / Ubuntu 20.04）：按子命令运行各模块。
# 用法：
#   ./scripts/main.sh keyboard            # 任务1 键盘遥控
#   ./scripts/main.sh avoid               # 任务2 深度避障
#   ./scripts/main.sh track               # 任务2 轨迹跟踪
#   ./scripts/main.sh explore             # 任务3 SLAM 同步探索
#   ./scripts/main.sh navigate 5 5        # 任务3 导航到 (5,5)
#   ./scripts/main.sh collect --n 2000   # 任务4 采集
#   ./scripts/main.sh train               # 任务4 训练
#   ./scripts/main.sh e2e                 # 任务4 推理端到端
set -e
cd "$(dirname "$0")/.."

MODE=${1:-help}
shift || true

case "$MODE" in
  keyboard) python3 -m modules.task1_keyboard.main "$@" ;;
  avoid)    python3 -m modules.task2_perception_control.main --mode avoid "$@" ;;
  track)    python3 -m modules.task2_perception_control.main --mode track "$@" ;;
  explore)  python3 -m modules.task3_slam_nav.main --mode explore "$@" ;;
  navigate) python3 -m modules.task3_slam_nav.main --mode navigate --goal "$@" ;;
  collect)  python3 -m modules.task4_e2e.collect_data "$@" ;;
  train)    python3 -m modules.task4_e2e.train "$@" ;;
  e2e)      python3 -m modules.task4_e2e.main "$@" ;;
  *)
    cat <<EOF
可用命令：
  keyboard | avoid | track | explore | navigate X Y | collect | train | e2e
EOF
    ;;
esac
