#!/usr/bin/env bash
# 依次跑 kp × kd 网格，最后汇总指标并生成整定图。
# 在包目录下执行：./run_eval.sh
set -euo pipefail

# 5 s 覆盖 ζ 最低（≈0.09）那一组的收敛时间 4/(ζ·ω_n) ≈ 2.4 s，
# 否则尾段平均取到的还不是稳态，稳态误差会被振荡污染。
DURATION=${DURATION:-5.0}
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
