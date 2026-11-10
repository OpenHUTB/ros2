#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_dynamic_safety_guard 统一入口；实现都在 ``uav_dynamic_safety_guard/``。

直接 ``python main.py --selftest`` 即可（只需 numpy），采集/训练另需
``mujoco`` 与同仓的 ``uav_mujoco_sim``。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# 未 colcon 安装时，同级目录里的 uav_mujoco_sim 直接加进搜索路径
_BASE = os.path.join(os.path.dirname(_HERE), "uav_mujoco_sim")
if os.path.isdir(_BASE) and _BASE not in sys.path:
    sys.path.append(_BASE)

from uav_dynamic_safety_guard.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
