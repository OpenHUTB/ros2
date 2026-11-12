#!/usr/bin/env bash
# 回放演示：需要先 `bash main.sh build` 生成 install/。
set -eo pipefail
cd "$(dirname "$0")"

if [ ! -f install/setup.bash ]; then
  echo "找不到 install/setup.bash，请先运行：bash main.sh build" >&2
  exit 2
fi
# shellcheck disable=SC1091
. install/setup.bash
exec ros2 launch telemetry_imputation main.launch.py project_root:="$PWD"
