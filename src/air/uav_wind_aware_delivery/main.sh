#!/usr/bin/env bash
set -e
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${UAV_PYTHON:-${UAV_VENV_PYTHON:-$HOME/ground_nn_nav/.venv/bin/python}}"
if [ ! -x "$PY" ]; then
  PY=python3
fi
exec "$PY" "$PKG/main.py" "$@"
