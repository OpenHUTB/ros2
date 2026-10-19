#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径点发布器统一入口（main.py）。

作业要求每个模块的入口为 ``main.*``；这里复用 ``waypoint_node`` 的节点逻辑，
使

    roslaunch waypoint_publisher main.launch
    rosrun   waypoint_publisher main.py

两种方式都能直接启动路径点发布与动态目标点切换。
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

from waypoint_node import main  # noqa: E402

if __name__ == "__main__":
    main()
