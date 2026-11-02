#!/usr/bin/env bash
# 本地 CI 脚本：与 GitHub Actions 工作流等价，一条命令跑完全部检查。
#
# 用法:
#   bash ci/run_ci.sh
#
# 退出码 0 表示全部通过。
set -euo pipefail

cd "$(dirname "$0")/.."
echo "=========================================="
echo " ROS2 Pure Pursuit - CI"
echo "=========================================="

echo
echo "[1/5] 语法检查"
find . -name "*.py" -not -path "./.git/*" -not -path "*/__pycache__/*" \
  | while read -r f; do
    python3 -m py_compile "$f"
  done
echo "  语法检查通过"

echo
echo "[2/5] 依赖检查"
python3 - <<'PY'
import importlib.util, sys
missing = [m for m in ("numpy", "matplotlib") if importlib.util.find_spec(m) is None]
if missing:
    print("  缺少依赖:", ", ".join(missing))
    print("  请执行: pip install -r requirements.txt")
    sys.exit(1)
print("  依赖齐全 (numpy, matplotlib)")
PY

echo
echo "[3/5] 单元测试"
python3 tests/test_core.py

echo
echo "[4/5] 仿真冒烟测试"
python3 sim/run_sim.py --path eight >/dev/null
python3 sim/run_sim.py --path circle >/dev/null
python3 sim/run_sim.py --path s_curve >/dev/null
python3 sim/run_sim.py --path straight >/dev/null
echo "  四条路径仿真全部通过"

echo
echo "[5/5] 基准矩阵"
python3 sim/benchmark.py >/dev/null
echo "  基准矩阵生成完成 -> results/benchmark.md"

echo
echo "=========================================="
echo " 全部检查通过"
echo "=========================================="
