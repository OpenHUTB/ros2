#!/bin/bash
# MuJoCo AEB —— 一键运行脚本（Linux / WSL / macOS）
#
#   ./main.sh                     离线批跑 + 导出 CSV/PNG（默认）
#   ./main.sh --gui               可视化窗口（需要图形界面）
#   ./main.sh --selftest          单元测试
#   ./main.sh --v-ego 12 --ttc 6  改工况与阈值
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(dirname "${SCRIPT_DIR}")"
cd "${PACKAGE_ROOT}"

# 虚拟机/无独显时用软件渲染（只有 --gui 才会用到）
export MJ_CORE_GLFW="${MJ_CORE_GLFW:-software}"

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

exec "$PYTHON_BIN" "${PACKAGE_ROOT}/main.py" "$@"
