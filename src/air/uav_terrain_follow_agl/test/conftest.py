# -*- coding: utf-8 -*-
"""让测试在源码目录直接跑：把包根与同级 uav_mujoco_sim 挂进 sys.path。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_SIBLING = os.path.normpath(os.path.join(_ROOT, "..", "uav_mujoco_sim"))
for _p in (_ROOT, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
