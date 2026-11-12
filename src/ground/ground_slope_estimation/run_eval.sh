#!/bin/bash
# 扫描「真实坡度 × 深度噪声」，每组写一个 CSV，最后汇总出图。
# 用法: bash run_eval.sh [输出目录]
set -e

OUT_DIR="${1:-results}"
SLOPES="${SLOPES:-0.0 5.0 10.0 15.0 20.0}"
NOISES="${NOISES:-0.0 0.005 0.010}"

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/scan_*.csv

for slope in $SLOPES; do
    for noise in $NOISES; do
        echo "== 坡度 ${slope}°，噪声 $(awk "BEGIN{printf \"%.0f\", $noise*1000}") mm =="
        ros2 run ground_slope_estimation slope_sim --ros-args \
            -p slope_deg:="$slope" \
            -p noise_std:="$noise" \
            -p out_dir:="$OUT_DIR"
    done
done

ros2 run ground_slope_estimation slope_report --results "$OUT_DIR" \
    --figure "$OUT_DIR/effect.png"
