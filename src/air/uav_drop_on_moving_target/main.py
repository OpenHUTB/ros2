#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键入口：解算 + MuJoCo 闭环复核三个算例。

    python main.py                  # 三个算例各真飞一次
    python main.py --case convoy_crosswind
    python main.py --no-sim         # 只跑算法层，秒级

源码目录直接运行不需要 colcon 安装：同级的 uav_mujoco_sim 由 flyer 自己挂
路径；安装后的入口是 ``ros2 run uav_drop_on_moving_target main``。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_drop_on_moving_target.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
