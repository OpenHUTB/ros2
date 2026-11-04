#!/usr/bin/env bash
# 2D 小车目标追踪 —— 一键运行脚本（Linux / WSL / macOS）
#
#   ./main.sh                         离线批跑 + 导出 CSV/PNG（默认）
#   ./main.sh --gui                   打开 pygame 窗口（需要 pygame + 图形界面）
#   ./main.sh --selftest              单元测试
#   ./main.sh --sim-time 60 --seed 7  改时长与随机种子
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "${PYTHON_BIN}" ]; then
  for candidate in python3 python; do
    if command -v "$candidate" >/dev/null 2>&1; then
      PYTHON_BIN="$candidate"
      break
    fi
  done
fi
if [ -z "${PYTHON_BIN}" ]; then
  echo "[错误] 找不到 python3/python，请先安装 Python 3.10+" >&2
  exit 1
fi

exec "$PYTHON_BIN" "$SCRIPT_DIR/main.py" "$@"
