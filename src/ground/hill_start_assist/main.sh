#!/bin/bash
# 模块入口包装：自动选渲染后端，把参数透传给 main.py
#   ./main.sh            跑自检
#   ./main.sh --gif      自检 + 出图（无 GPU 时自动 MUJOCO_GL=osmesa）
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${MUJOCO_GL:-}" ] && [ -z "${DISPLAY:-}" ]; then
  export MUJOCO_GL=osmesa
fi
exec python3 "$HERE/main.py" "$@"
