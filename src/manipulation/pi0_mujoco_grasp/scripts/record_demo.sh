#!/bin/bash
# Record the MuJoCo pick-and-place demo and build stills / GIF / MP4.
# Usage: bash scripts/record_demo.sh [output_dir]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MODULE=$(cd "$HERE/.." && pwd)
OUT=${1:-/tmp/pi0_frames}
FPS=${FPS:-30}

cd "$MODULE"
if [ -d .venv ]; then source .venv/bin/activate; fi
export DISPLAY=${DISPLAY:-:0}
# 按当前 UID 自动查找 Xauthority（非 1000 用户也能用）
export XAUTHORITY=${XAUTHORITY:-$(ls /run/user/$(id -u)/gdm/Xauthority 2>/dev/null || echo "$HOME/.Xauthority")}
export MUJOCO_GL=${MUJOCO_GL:-glfw}

python3 -m pi0_mujoco_grasp.app --record "$OUT" --fps "$FPS"
python3 scripts/make_gif.py --frames "$OUT" --out "$MODULE/results" --fps "$FPS"
ls -la "$MODULE/results"
