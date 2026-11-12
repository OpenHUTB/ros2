#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""源码目录直接运行入口：python main.py [--no-sim] [--sweep] [--json]"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_terrain_follow_agl.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
