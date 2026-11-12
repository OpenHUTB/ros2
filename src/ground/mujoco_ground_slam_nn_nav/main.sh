#!/usr/bin/env bash
# 统一入口的脚本包装：优先使用模块内 .venv 的 Python 3.11。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$ROOT/.venv/bin/python"
if [ ! -x "$PY" ]; then
  PY="python3"
fi
exec "$PY" "$ROOT/main.py" "$@"
