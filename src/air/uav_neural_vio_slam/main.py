#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_neural_vio_slam 统一入口。薄封装，逻辑在 ``uav_neural_vio_slam/cli.py``。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_neural_vio_slam.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
