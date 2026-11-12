#!/usr/bin/env bash
# 四阶段自检：语法 -> 依赖 -> 单元测试 -> 离线闭环
# 用法：bash ci/run_ci.sh        （在模块目录下执行）
set -euo pipefail

cd "$(dirname "$0")/.."
PY=${PYTHON:-python3}
fail() { echo "[FAIL] $1"; exit 1; }
ok() { echo "[ OK ] $1"; }

echo "== 1/4 语法检查 =="
$PY -m compileall -q src sim tests main.py >/dev/null || fail "compileall 失败"
ok "所有 .py 均可编译"

echo "== 2/4 依赖检查 =="
$PY - <<'EOF' || exit 1
import importlib.util as u
missing = [m for m in ("numpy",) if u.find_spec(m) is None]
if missing:
    print("缺少依赖:", missing)
    raise SystemExit(1)
print("numpy", __import__("numpy").__version__)
EOF
ok "核心依赖齐全"

echo "== 3/4 单元测试 =="
$PY -m pytest tests -q || fail "单元测试未通过"
ok "单元测试通过"

echo "== 4/4 离线闭环 =="
$PY main.py --no-sweep >/dev/null || fail "离线闭环失败"
ok "6 个场景闭环跑通"

echo "全部检查通过。"
