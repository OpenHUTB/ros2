#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 入口：演示用"真值 + 带误差里程计"发布器（roslaunch 用）。

不需要任何仿真器，用来配合 ``odom_eval_node.py`` 独立演示 ATE/RPE 的计算过程。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from traj_eval.ros_sim_publisher import main  # noqa: E402

if __name__ == "__main__":
    main()
