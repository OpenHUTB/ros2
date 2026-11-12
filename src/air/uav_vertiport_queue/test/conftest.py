#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pytest 路径引导：源码目录直接运行时也能 import 本包与仿真底座。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
