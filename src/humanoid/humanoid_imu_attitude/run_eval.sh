#!/usr/bin/env bash
# 依次跑三种动态强度，最后汇总指标并画对比图。
# 在包目录下执行：DURATION=20 ./run_eval.sh
set -euo pipefail

DURATION=${DURATION:-20}
OUT_DIR=${OUT_DIR:-results}
PROFILES=${PROFILES:-"static slow fast"}

mkdir -p "$OUT_DIR"
for profile in $PROFILES; do
  echo "=== profile: $profile ==="
  ros2 launch humanoid_imu_attitude main.launch.py \
    profile:="$profile" duration:="$DURATION" out_dir:="$OUT_DIR"
done

CURVE_PROFILE=$(echo "$PROFILES" | awk '{print $NF}')
ros2 run humanoid_imu_attitude attitude_eval \
  --results "$OUT_DIR" --figure "$OUT_DIR/effect.png" --profile "$CURVE_PROFILE"
