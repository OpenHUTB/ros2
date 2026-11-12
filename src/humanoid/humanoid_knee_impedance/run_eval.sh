#!/usr/bin/env bash
# 依次跑 kp × kd 网格，最后汇总指标并生成整定图。
# 在包目录下执行：./run_eval.sh
set -euo pipefail

DURATION=${DURATION:-2.0}
OUT_DIR=${OUT_DIR:-results}
KP_LIST=${KP_LIST:-"2.0 5.0 10.0"}
KD_LIST=${KD_LIST:-"0.10 0.50 1.00"}

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/samples_kp*_kd*.csv "$OUT_DIR"/metrics.csv
for kp in $KP_LIST; do
  for kd in $KD_LIST; do
    echo "=== kp: $kp N·m/rad, kd: $kd N·m·s/rad ==="
    ros2 launch humanoid_knee_impedance main.launch.py \
      knee_kp:="$kp" knee_kd:="$kd" \
      duration:="$DURATION" out_dir:="$OUT_DIR"
  done
done

ros2 run humanoid_knee_impedance knee_report \
  --results "$OUT_DIR" --figure "$OUT_DIR/effect.png"
