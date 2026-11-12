#!/usr/bin/env bash
set -e
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${UAV_PYTHON:-${UAV_VENV_PYTHON:-$PKG/.venv/bin/python}}"
if [ ! -x "$PY" ]; then
  echo "[uav] 找不到 Python: $PY" >&2
  echo "  请先在包目录运行: bash scripts/setup_env.sh" >&2
  echo "  或设置: export UAV_VENV_PYTHON=/path/to/python3.11" >&2
  exit 1
fi
exec "$PY" "$PKG/main.py" "$@"
