#!/usr/bin/env bash
# uav_vertiport_queue 的可重复验证入口（CI 用）
#
# 依赖只有 python3 + numpy / matplotlib / pyyaml / pytest。
# 不需要 ROS 2，也不需要 MuJoCo：闭环冒烟测试里用
# pytest.importorskip("mujoco") 自动跳过，所以最小 CI 镜像
# 也能把算法层（排队 / 容量扫描 / 轨迹跟踪 / 场景参数）跑完。
#
# 用法：
#   bash tools/ci_tests.sh
#   PYTHON=/path/to/venv/bin/python bash tools/ci_tests.sh
#   bash tools/ci_tests.sh -k queue        # 透传 pytest 参数
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  if command -v python3 >/dev/null 2>&1; then
    PY=python3
  else
    PY=python
  fi
fi

echo "== 环境 =="
"$PY" -c "import numpy, matplotlib, yaml, pytest; print('numpy', numpy.__version__, '| matplotlib', matplotlib.__version__, '| pyyaml', yaml.__version__, '| pytest', pytest.__version__)"

echo "== 测试 =="
"$PY" -m pytest test/ -q "$@"
