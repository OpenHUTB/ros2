#!/usr/bin/env bash
# 统一入口：check / run / bench / gif
set -e
cd "$(dirname "$0")"
exec python3 main.py "$@"
