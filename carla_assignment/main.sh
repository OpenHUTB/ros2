#!/usr/bin/env bash
# ============================================================
#  CARLA 0.9.16 作业一 - Ubuntu/WSL 入口（键盘运动控制）
#  用法:
#    bash main.sh control [--sim_time N] [--follow]
#  需先在另一终端启动 CARLA 服务端:
#    ./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low
# ============================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "[用法] bash main.sh <task> [args...]"
  echo "  tasks: control"
  exit 1
}

[ $# -ge 1 ] || usage
TASK="$1"; shift

case "$TASK" in
  control)     python3 "$ROOT/01_control/main.py" "$@" ;;
  *) echo "[错误] 未知任务: $TASK"; usage ;;
esac
