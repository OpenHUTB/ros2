#!/usr/bin/env bash
# 无参数时跑自检；其余参数原样透传给 main.py。
# 提示：提交前执行 `git update-index --chmod=+x main.sh` 保证可执行位。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -eq 0 ]; then
    exec python3 "${SCRIPT_DIR}/main.py" --selftest
fi

exec python3 "${SCRIPT_DIR}/main.py" "$@"
