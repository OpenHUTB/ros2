#!/usr/bin/env python3
"""模块入口：``python main.py demo`` 生成效果图，``python main.py sim`` 连模拟器实测。"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tire_friction_braking_sweep.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
