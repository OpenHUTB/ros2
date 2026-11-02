#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 节点入口（catkin_install_python 需要 scripts/ 下的可执行脚本）。

实际逻辑在 ``lateral_control_benchmark/ros_node.py``；这里只做路径注入与转发，
这样同一份代码既能用 ``python3 main.py --mode ros`` 运行，也能被 roslaunch 直接启动。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from lateral_control_benchmark.ros_node import main  # noqa: E402

if __name__ == "__main__":
    main()
