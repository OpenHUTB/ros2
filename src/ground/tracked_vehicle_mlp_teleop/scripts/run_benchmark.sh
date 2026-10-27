#!/usr/bin/env bash
# 在 3 个场景中依次运行 RAW / PI / MLP 评估，并汇总指标与绘图。
set -euo pipefail

PKG="$(rospack find tracked_vehicle_mlp_teleop)"
mkdir -p "${PKG}/data" "${PKG}/media"

for world in flat slope low_friction; do
  spawn_x=0.0
  if [ "${world}" = "slope" ]; then
    spawn_x=-2.0
  fi
  echo "===== 评估 ${world} ====="
  timeout 240 roslaunch tracked_vehicle_mlp_teleop eval.launch \
    world:="${world}" spawn_x:="${spawn_x}" \
    output_csv:="${PKG}/data/eval_${world}.csv" \
    summary_csv:="${PKG}/data/summary_${world}.csv" || true
done

python3 "${PKG}/scripts/evaluate.py" \
  --input "${PKG}/data/summary_flat.csv" \
          "${PKG}/data/summary_slope.csv" \
          "${PKG}/data/summary_low_friction.csv" \
  --output-csv "${PKG}/data/summary_all.csv" \
  --output-md "${PKG}/data/summary_table.md" \
  --output-fig "${PKG}/media/metrics.png"

echo "===== 完成：${PKG}/media/metrics.png ====="
