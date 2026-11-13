#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末端物流多站点配送与换电调度 —— 一键运行入口。

用法::

    python main.py                       # 跑全部 case（含 MuJoCo 闭环真飞）
    python main.py --case case_farm_dense
    python main.py --no-sim              # 仅算法层（秒级，不需要 mujoco）

源码目录直接运行时不需要 colcon install：本文件会把同级的 ``uav_mujoco_sim``
底座加入 ``sys.path``。装好包之后对应入口是 ``ros2 run uav_delivery_route main``。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)


def main():
    from uav_delivery_route.cli import main as _main
    return _main()


if __name__ == "__main__":
    raise SystemExit(main())
