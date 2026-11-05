#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_mujoco_sim 统一入口（main 开头）：薄封装，逻辑在 ``uav_mujoco_sim/cli.py``。

源码目录 ``python main.py …``、安装后的 ``share/uav_mujoco_sim/main.py``、
``ros2 run uav_mujoco_sim main`` 共用这一份代码；import 路径按布局自动补齐。
"""
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
#: 包可能在的位置：源码目录；安装布局的 <prefix>/lib/pythonX.Y/site-packages
_INSTALL_PREFIX = os.path.dirname(os.path.dirname(_HERE))
_ROOTS = [_HERE] + sorted(
    glob.glob(os.path.join(_INSTALL_PREFIX, "lib", "python*", "site-packages")))
_ROOTS = [p for p in _ROOTS if os.path.isdir(os.path.join(p, "uav_mujoco_sim"))]
for _p in reversed(_ROOTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from uav_mujoco_sim.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
