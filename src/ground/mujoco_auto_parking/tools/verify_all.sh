#!/usr/bin/env bash
# 自动泊车模块全量回归：所有任务入口逐个运行，记录日志和退出码。
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
DOCS_DIR="${AUTO_PARKING_DOCS_DIR:-$REPO_ROOT/docs}"
OUT_DIR="$DOCS_DIR/evidence/mujoco_auto_parking/verification"
mkdir -p "$OUT_DIR"
: > "$OUT_DIR/summary.txt"

export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
export PYTHONUNBUFFERED=1
cd "$PACKAGE_DIR"

run_case() {
    local name="$1"
    shift
    echo "== $name =="
    bash ./main.sh "$@" > "$OUT_DIR/$name.log" 2>&1
    local code=$?
    echo "$name exit=$code" | tee -a "$OUT_DIR/summary.txt"
    return $code
}

FAILED=0
run_case task1 --test || FAILED=1
run_case sensors --test-sensors || FAILED=1
run_case networks --test-nn || FAILED=1
run_case trajectory --eval-trajectory || FAILED=1
run_case task3 --test3 || FAILED=1
run_case task4 --test4 || FAILED=1

echo "---- summary ----"
cat "$OUT_DIR/summary.txt"
if [ "$FAILED" -ne 0 ]; then
    echo "回归失败：请查看 $OUT_DIR"
    exit 1
fi
echo "全部入口通过。"
