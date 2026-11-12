#!/usr/bin/env bash
# 统一入口：转发给 scripts/main.py
# 用法: ./main.sh [--mode nn|expert|manual] [--save-gif out.gif] [其他 main.py 参数]
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$HERE/scripts/main.py" "$@"
