#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_slam_nav 统一入口。源码目录直接 python main.py 与安装后 ros2 run 走同一份代码。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_slam_nav.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
