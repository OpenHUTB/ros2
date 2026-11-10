# -*- coding: utf-8 -*-
"""pytest 引导：把本包与底座 uav_mujoco_sim 放进 sys.path。

单测只依赖 numpy，不需要 MuJoCo / ROS；需要真值几何的用例直接构造几何体。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_AIR = os.path.dirname(_PKG)

for p in (_PKG, os.path.join(_AIR, "uav_mujoco_sim")):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
