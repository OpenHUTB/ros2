#!/usr/bin/env bash
# 依次运行多个 QoS 场景，每个场景重复 REPEATS 次（结果写
# results/<profile>.r<N>.csv），最后按场景取中位数汇总成对比表。
#
# 需在包目录下执行，否则 results/ 会落到当前工作目录：
#   DURATION=10 RATE=200 REPEATS=3 bash run_sweep.sh
#   PROFILES="reliable_depth10 best_effort_depth1" bash run_sweep.sh
#
# REPEATS 默认 3：单次结果对系统负载敏感（冷启动未预热时曾丢 24%），
# 单次值不足以代表该配置，取中位数更稳。

set -uo pipefail

PKG=joint_qos_latency_eval
DURATION=${DURATION:-10}
RATE=${RATE:-200}
REPEATS=${REPEATS:-3}
OUT=${OUT:-results}
PROFILES=${PROFILES:-"best_effort_depth10 best_effort_depth1 reliable_depth10 reliable_depth1 reliable_transient_local"}

# 应发总条数：作为 missing/drop% 的分母，末尾丢包与欠触发都会被计入
EXPECTED=$(awk -v r="$RATE" -v d="$DURATION" 'BEGIN{printf "%d", r * d}')

mkdir -p "$OUT"
# 清掉上一轮的结果，避免历史 CSV 混进本次汇总
rm -f "$OUT"/*.csv

failed=""
for profile in $PROFILES; do
  echo "=== $profile ==="
  for repeat in $(seq 1 "$REPEATS"); do
    echo "--- $profile r$repeat/$REPEATS ---"
    # 外层 timeout 兜底：节点若因 DDS/rclpy 异常卡住，也不会把整个脚本拖死
    timeout -k 2 $((DURATION + 20)) ros2 run "$PKG" joint_qos_sub --ros-args \
      -p qos_profile:="$profile" \
      -p duration:="$((DURATION + 2))" \
      -p csv_path:="$OUT/$profile.r$repeat.csv" \
      -p expected_msgs:="$EXPECTED" \
      > "$OUT/$profile.r$repeat.sub.log" 2>&1 &
    sub_pid=$!
    sleep 1
    timeout -k 2 $((DURATION + 20)) ros2 run "$PKG" joint_qos_pub --ros-args \
      -p qos_profile:="$profile" \
      -p duration:="$DURATION" \
      -p rate_hz:="$RATE" \
      > "$OUT/$profile.r$repeat.pub.log" 2>&1
    pub_rc=$?
    wait "$sub_pid"
    sub_rc=$?
    if [ "$pub_rc" -ne 0 ] || [ "$sub_rc" -ne 0 ]; then
      echo "!!! $profile r$repeat failed: pub rc=$pub_rc sub rc=$sub_rc," \
        "see $OUT/$profile.r$repeat.*.log" >&2
      failed="$failed $profile.r$repeat"
    fi
  done
done

# 只汇总本次跑过的场景，不用通配符，免得把目录里其他 CSV 一起算进去
csvs=""
for profile in $PROFILES; do
  for csv in "$OUT/$profile".r*.csv; do
    if [ -f "$csv" ]; then
      csvs="$csvs $csv"
    fi
  done
done
if [ -z "$csvs" ]; then
  echo 'no CSV produced, nothing to summarize' >&2
  exit 1
fi

ros2 run "$PKG" qos_summarize --median --expected "$EXPECTED" $csvs

if [ -n "$failed" ]; then
  echo "failed runs:$failed" >&2
  exit 1
fi
