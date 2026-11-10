#!/usr/bin/env bash
# 依次运行多个 QoS 场景，每个场景单独起进程（发布端/订阅端不共享内存），
# 结果写入 results/<profile>.csv，最后汇总成对比表。
#
#   DURATION=10 RATE=200 ./run_sweep.sh
#   PROFILES="reliable_depth10 best_effort_depth1" ./run_sweep.sh

set -uo pipefail

PKG=joint_qos_latency_eval
DURATION=${DURATION:-10}
RATE=${RATE:-200}
OUT=${OUT:-results}
PROFILES=${PROFILES:-"best_effort_depth10 best_effort_depth1 reliable_depth10 reliable_depth1"}

mkdir -p "$OUT"
# 清掉上一轮的结果，避免历史 CSV 混进本次汇总
rm -f "$OUT"/*.csv

failed=""
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
  pub_rc=$?
  wait "$sub_pid"
  sub_rc=$?
  if [ "$pub_rc" -ne 0 ] || [ "$sub_rc" -ne 0 ]; then
    echo "!!! $profile failed: pub rc=$pub_rc sub rc=$sub_rc, see $OUT/$profile.*.log" >&2
    failed="$failed $profile"
  fi
done

# 只汇总本次跑过的场景，不用通配符，免得把目录里其他 CSV 一起算进去
csvs=""
for profile in $PROFILES; do
  if [ -f "$OUT/$profile.csv" ]; then
    csvs="$csvs $OUT/$profile.csv"
  fi
done
if [ -z "$csvs" ]; then
  echo 'no CSV produced, nothing to summarize' >&2
  exit 1
fi

ros2 run "$PKG" qos_summarize $csvs

if [ -n "$failed" ]; then
  echo "failed profiles:$failed" >&2
  exit 1
fi
