#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""输电走廊无人机巡检覆盖规划 —— 一键运行入口。

用法::

    python main.py                        # 跑全部 case（含 MuJoCo 闭环真飞）
    python main.py --case case_grid_short # 只跑短走廊
    python main.py --no-sim               # 仅算法层（秒级，不需要 mujoco）
    python main.py --json PATH            # 结果写入指定 JSON（默认 results/ 下）

源码目录直接运行不需要 colcon install：本文件会把同级的 ``uav_mujoco_sim``
加入 ``sys.path``。装好包之后对应入口是 ``ros2 run uav_inspection_cover main``。
"""
from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING = os.path.normpath(os.path.join(_HERE, "..", "uav_mujoco_sim"))
for _p in (_HERE, _SIBLING):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_inspection_cover.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
