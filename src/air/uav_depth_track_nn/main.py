#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_depth_track_nn 统一入口（作业要求的 main. 开头入口）。

薄封装，逻辑在 ``uav_depth_track_nn/main.py``：源码目录里直接
``python main.py`` 与安装后的 ``ros2 run uav_depth_track_nn main``
走同一份代码。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_depth_track_nn.main import main  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(main())
