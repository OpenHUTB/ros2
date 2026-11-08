#!/usr/bin/env bash
# CI entry point for uav_noise_footprint.
#
# Needs only python3 plus: pip install numpy matplotlib pyyaml pytest
# No ROS installation and no MuJoCo are required: closed-loop cases
# self-skip when mujoco is unavailable (pytest.importorskip), so a minimal
# CI image still exercises the acoustic / exposure / mitigation layers.
#
# Usage:  bash tools/ci_tests.sh
#         PYTHON=/path/to/venv/bin/python bash tools/ci_tests.sh
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  if command -v python3 >/dev/null 2>&1; then PY=python3; else PY=python; fi
fi

"$PY" -c "import numpy, matplotlib, yaml, pytest; print('numpy', numpy.__version__, '| matplotlib', matplotlib.__version__, '| pytest', pytest.__version__)"
"$PY" -m pytest test/ -q "$@"
