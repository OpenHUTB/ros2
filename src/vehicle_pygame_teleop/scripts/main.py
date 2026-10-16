#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆键盘遥操作统一入口（main.py）。

作业要求每个模块的入口为 ``main.*``；这里复用 ``vehicle_pygame_teleop``
的节点逻辑，使

    roslaunch vehicle_pygame_teleop main.launch
    rosrun   vehicle_pygame_teleop main.py

两种方式都能直接启动键盘遥操作与内置二维模拟。
"""
from __future__ import absolute_import, print_function

import os
import sys

# 兼容 devel / install 两种布局：catkin 的 relay 脚本有时会把 __file__ 指到
# lib/<pkg>/ 下，因此除 abspath 外再补一次 realpath（会解析软链）。
for _path in (os.path.abspath(__file__), os.path.realpath(__file__)):
    _dir = os.path.dirname(_path)
    if _dir not in sys.path:
        sys.path.insert(0, _dir)

from vehicle_pygame_teleop import main  # noqa: E402

if __name__ == "__main__":
    main()
