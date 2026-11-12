#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低空起降场进离场排队与容量评估 —— 一键运行入口。

用法::

    python main.py                      # 全部场景 + 容量扫描 + MuJoCo 闭环
    python main.py --scenario light     # 只跑一个场景
    python main.py --no-sim             # 仅算法层（秒级，不需要 mujoco）

源码目录直接运行时不需要 colcon install：本文件会把同级的
``uav_mujoco_sim`` 底座加入 ``sys.path``。装好包之后对应入口是
``ros2 run uav_vertiport_queue main``。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_vertiport_queue.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
