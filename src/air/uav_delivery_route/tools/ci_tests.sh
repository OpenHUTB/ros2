#!/usr/bin/env bash
# CI entry point for uav_delivery_route.
#
# Only needs python3 plus: pip install numpy matplotlib pyyaml pytest
# No ROS installation and no MuJoCo are required: the closed-loop smoke test
# is guarded by pytest.importorskip("mujoco"), so it self-skips on a minimal
# image while planner / energy model / route checker tests still run.
#
# Usage:  bash tools/ci_tests.sh
#         PYTHON=/path/to/venv/bin/python bash tools/ci_tests.sh -k planner
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  if command -v python3 >/dev/null 2>&1; then PY=python3; else PY=python; fi
fi

"$PY" -c "import numpy, matplotlib, yaml, pytest; print('numpy', numpy.__version__, '| matplotlib', matplotlib.__version__, '| pyyaml', yaml.__version__, '| pytest', pytest.__version__)"
"$PY" -m pytest test/ -q "$@"
