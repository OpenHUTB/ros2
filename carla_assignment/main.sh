#!/usr/bin/env bash
# ============================================================
#  CARLA 0.9.16 无人车神经网络四次作业 - Ubuntu/WSL 入口
#  用法:
#    bash main.sh control              : 作业一
#    bash main.sh perception --mode train  : 作业二 训练NN
#    bash main.sh perception --mode run    : 作业二 在线
#    bash main.sh navigation --mode train  : 作业三 训练NN
#    bash main.sh navigation --mode run --goal 20,8
#    bash main.sh end_to_end --mode test
#  需先在另一终端启动 CARLA 服务端:
#    ./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low
# ============================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "[用法] bash main.sh <task> [args...]"
  echo "  tasks: control / perception / navigation / end_to_end"
  exit 1
}

[ $# -ge 1 ] || usage
TASK="$1"; shift

case "$TASK" in
  control)     python3 "$ROOT/01_control/main.py" "$@" ;;
  perception)  python3 "$ROOT/02_perception/main.py" "$@" ;;
  navigation)  python3 "$ROOT/03_navigation/main.py" "$@" ;;
  end_to_end)  python3 "$ROOT/04_end_to_end/main.py" "$@" ;;
  *) echo "[错误] 未知任务: $TASK"; usage ;;
esac
