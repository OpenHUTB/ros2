#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Spray coverage & deposition assessment -- one-shot entry.

用法::

    python main.py                  # 全部 case + MuJoCo 闭环复飞
    python main.py --case case_field_rect
    python main.py --no-sim         # 仅算法层（秒级，不需要 mujoco）

源码目录直接运行时不需要 colcon install：本文件把同级的 ``uav_mujoco_sim``
底座加入 ``sys.path``。装好包之后对应入口是 ``ros2 run uav_agri_spray main``。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_agri_spray.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
