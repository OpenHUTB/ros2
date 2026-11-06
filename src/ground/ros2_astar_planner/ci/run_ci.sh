#!/usr/bin/env bash
# 一键自检：语法 -> 依赖 -> 单元测试 -> 离线仿真 -> 基准矩阵
# 不依赖 ROS，可在任何装有 Python 3.9+ 与 NumPy / Matplotlib 的机器上运行。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

step() { echo; echo "=== $* ==="; }

step "1/5 语法检查"
python3 -m compileall -q src sim tests launch >/dev/null
echo "语法检查通过"

step "2/5 依赖检查"
python3 - <<'PY'
import importlib, sys
missing = []
for mod in ("numpy", "matplotlib"):
    try:
        importlib.import_module(mod)
    except ImportError:
        missing.append(mod)
if missing:
    print("缺少依赖：", ", ".join(missing))
    sys.exit(1)
print("依赖检查通过（numpy / matplotlib 可用）")
PY

step "3/5 单元测试"
python3 tests/test_core.py

step "4/5 离线仿真"
for m in corridor maze random; do
  python3 sim/run_sim.py --map "$m" --out "results/${m}.png"
done

step "5/5 基准矩阵"
python3 sim/benchmark.py >/dev/null
echo "基准矩阵已生成：results/benchmark.md"
grep -c "^|" results/benchmark.md >/dev/null

echo
echo "全部检查通过 ✅"
