#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""向后兼容入口：`rosrun mujoco_vehicle_aeb main.py` 仍可用。

实现已移到 `scripts/cli.py`（默认离线批跑，`--gui` 才开窗口）。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cli import main  # noqa: E402

if __name__ == "__main__":
    main()
