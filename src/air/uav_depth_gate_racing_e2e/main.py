#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_depth_gate_racing_e2e 统一入口。逻辑在 ``uav_depth_gate_racing_e2e/cli.py``。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_depth_gate_racing_e2e.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
