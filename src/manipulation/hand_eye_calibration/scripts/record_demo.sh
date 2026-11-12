#!/bin/bash
# Record the hand-eye calibration demo and build screenshots / GIF / MP4 / error plot.
# Usage: bash scripts/record_demo.sh [output_dir]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MODULE=$(cd "$HERE/.." && pwd)
OUT=${1:-/tmp/he_frames}

cd "$MODULE"
if [ -d .venv ]; then source .venv/bin/activate; fi
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$(ls /run/user/$(id -u)/gdm/Xauthority 2>/dev/null || echo "$HOME/.Xauthority")}
export MUJOCO_GL=${MUJOCO_GL:-glfw}

python3 -m hand_eye_calibration.app --record "$OUT" --fps 20 --stills results
python3 scripts/make_gif.py --frames "$OUT" --out results
python3 scripts/plot_result.py --log results/calibration_result.json --out results/calibration_error.png
ls -la results
