#!/usr/bin/env bash
# 依次跑两种控制器 × 各跌落高度，最后汇总指标并生成对比图。
# 在包目录下执行：./run_eval.sh
set -euo pipefail

DURATION=${DURATION:-3.0}
OUT_DIR=${OUT_DIR:-results}
MODES=${MODES:-"stiff impedance"}
DROPS=${DROPS:-"0.03 0.05 0.10"}

mkdir -p "$OUT_DIR"
for mode in $MODES; do
  for drop in $DROPS; do
    echo "=== 控制器: $mode, 跌落高度: ${drop} m ==="
    ros2 launch humanoid_landing_impedance main.launch.py \
      controller_mode:="$mode" drop_height:="$drop" \
      duration:="$DURATION" out_dir:="$OUT_DIR"
  done
done

ros2 run humanoid_landing_impedance landing_report \
  --results "$OUT_DIR" --figure "$OUT_DIR/effect.png"
