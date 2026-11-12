#!/usr/bin/env bash
# MuJoCo 两轮自平衡小车 —— 一键运行脚本（Linux / WSL / macOS）
#
#   ./main.sh                      # 离线批跑 + 导出 CSV/PNG（默认）
#   ./main.sh --gui                # 可视化窗口（需要图形界面）
#   ./main.sh --selftest           # 单元测试
#   ./main.sh --lean 0.2 --duration 8 --save_dir shots
#
# 说明：脚本只负责"挑一个能用的 Python 并转发参数"，逻辑都在 main.py 里。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "$PYTHON_BIN" ]; then
  for candidate in python3 python; do
    if command -v "$candidate" >/dev/null 2>&1; then
      PYTHON_BIN="$candidate"
      break
    fi
  done
fi

if [ -z "$PYTHON_BIN" ]; then
  echo "[错误] 找不到 python3/python，请先安装 Python 3.10+" >&2
  exit 1
fi

exec "$PYTHON_BIN" "$SCRIPT_DIR/main.py" "$@"
