#!/usr/bin/env bash
# uav_deconflict 的可重复验证入口（CI 用）
#
# 依赖只有 python3 + numpy + pytest。不需要 ROS 2，也不需要 MuJoCo：
# ROS 封装层的测试在没装 rclpy 时自动跳过，MuJoCo 冒烟测试用
# pytest.importorskip("mujoco") 自动跳过，所以最小 CI 镜像也能把
# 算法层（冲突检测 / 解脱规划 / 在线监视 / 场景参数）跑完。
#
# 源码树直接运行时需要同仓库的 src/air/uav_mujoco_sim 存在
# （conftest.py / main.py 会自动把它加入 sys.path）。
#
# 用法：
#   bash tools/ci_tests.sh
#   PYTHON=/path/to/venv/bin/python bash tools/ci_tests.sh
#   bash tools/ci_tests.sh -k resolve     # 透传 pytest 参数
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
"$PY" -c "import numpy, pytest; print('numpy', numpy.__version__, '| pytest', pytest.__version__)"

echo "== 测试 =="
"$PY" -m pytest test/ -q "$@"

echo "== 冒烟：算法层端到端（--no-sim） =="
"$PY" main.py --no-sim
