#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 入口：轨迹精度在线评估节点（roslaunch 用）。

实际实现在 ``traj_eval/ros_node.py``；这里只做路径注入与转发，
保证既能 ``roslaunch`` 启动，也能在源码目录用 ``python3 main.py --mode ros`` 运行。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from traj_eval.ros_node import main  # noqa: E402

if __name__ == "__main__":
    main()
