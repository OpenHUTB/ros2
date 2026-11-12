#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UAV emergency supply drop — one-shot entry.

Usage::

    python main.py                       # all cases incl. MuJoCo closed loop
    python main.py --case case_aid_wind  # single case
    python main.py --no-sim              # algorithm layer only, seconds
    python main.py --json out.json       # also write combined JSON

Running from the source tree needs no colcon install: the sibling
``uav_mujoco_sim`` base is appended to ``sys.path`` here. After install the
entry point is ``ros2 run uav_emergency_drop main``.
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_emergency_drop.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
