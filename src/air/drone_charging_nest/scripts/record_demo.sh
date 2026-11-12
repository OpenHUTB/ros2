#!/bin/bash
# Record the drone auto-charging-nest demo and build screenshots / GIF / MP4 / plot.
# Usage: bash scripts/record_demo.sh [output_dir]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MODULE=$(cd "$HERE/.." && pwd)
OUT=${1:-/tmp/dc_frames}

cd "$MODULE"
if [ -d .venv ]; then source .venv/bin/activate; fi
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$(ls /run/user/$(id -u)/gdm/Xauthority 2>/dev/null || echo "$HOME/.Xauthority")}
export MUJOCO_GL=${MUJOCO_GL:-glfw}

python3 -m drone_charging_nest.app --record "$OUT" --fps 20 --log results/mission_log.json --stills results
python3 scripts/make_gif.py --frames "$OUT" --out results
python3 scripts/plot_mission.py --log results/mission_log.json --out results/mission_curve.png
ls -la results
