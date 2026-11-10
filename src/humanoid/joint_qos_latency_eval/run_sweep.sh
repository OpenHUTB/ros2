#!/usr/bin/env bash
# 依次运行多个 QoS 场景，每个场景单独起进程（发布端/订阅端不共享内存），
# 结果写入 results/<profile>.csv，最后汇总成对比表。
#
#   DURATION=10 RATE=200 ./run_sweep.sh
#   PROFILES="reliable_depth10 best_effort_depth1" ./run_sweep.sh

set -u

PKG=joint_qos_latency_eval
DURATION=${DURATION:-10}
RATE=${RATE:-200}
OUT=${OUT:-results}
PROFILES=${PROFILES:-"best_effort_depth10 best_effort_depth1 reliable_depth10 reliable_depth1"}

mkdir -p "$OUT"

for profile in $PROFILES; do
  echo "=== $profile ==="
  # 外层 timeout 兜底：节点若因 DDS/rclpy 异常卡住，也不会把整个脚本拖死
  timeout -k 2 $((DURATION + 20)) ros2 run "$PKG" joint_qos_sub --ros-args \
    -p qos_profile:="$profile" \
    -p duration:="$((DURATION + 2))" \
    -p csv_path:="$OUT/$profile.csv" \
    > "$OUT/$profile.sub.log" 2>&1 &
  sub_pid=$!
  sleep 1
  timeout -k 2 $((DURATION + 20)) ros2 run "$PKG" joint_qos_pub --ros-args \
    -p qos_profile:="$profile" \
    -p duration:="$DURATION" \
    -p rate_hz:="$RATE" \
    > "$OUT/$profile.pub.log" 2>&1
  wait "$sub_pid"
done

ros2 run "$PKG" qos_summarize "$OUT"/*.csv
