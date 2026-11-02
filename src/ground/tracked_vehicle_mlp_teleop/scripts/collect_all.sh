#!/usr/bin/env bash
# 依次在 3 个场景中采集 60 秒 PI 专家数据。
# 使用前先 source devel/setup.bash，并确保没有其他 Gazebo 实例占用端口。
set -euo pipefail

PKG="$(rospack find tracked_vehicle_mlp_teleop)"
DURATION="${DURATION:-60}"
mkdir -p "${PKG}/data"

for world in flat slope low_friction; do
  spawn_x=0.0
  if [ "${world}" = "slope" ]; then
    spawn_x=-2.0
  fi
  echo "===== 采集 ${world} (${DURATION}s) ====="
  timeout 180 roslaunch tracked_vehicle_mlp_teleop collect.launch \
    world:="${world}" spawn_x:="${spawn_x}" duration:="${DURATION}" \
    output_csv:="${PKG}/data/collect_${world}.csv" || true
done

echo "===== 训练 MLP ====="
python3 "${PKG}/scripts/train_mlp.py" \
  --csv "${PKG}/data/collect_flat.csv" \
        "${PKG}/data/collect_slope.csv" \
        "${PKG}/data/collect_low_friction.csv" \
  --output "${PKG}/models/mlp_policy.pt" \
  --norm-output "${PKG}/models/norm_stats.npz"
