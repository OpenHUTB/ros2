#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_control_drone 模块统一入口（main.py）。

作业约定：每个模块的入口必须是 ``main.`` 开头，且 ``roslaunch`` 与 ``rosrun``
两种方式都能直接跑起来整个模块::

    roslaunch gesture_control_drone main.launch    # 作业要求的 launch 启动
    rosrun   gesture_control_drone main.py         # 只跑手势感知节点

入口本身不含逻辑，只把节点逻辑转发过去，保证两种启动方式行为一致；
无人机控制节点由 launch 以 ``type="drone_control_node.py"`` 启动
（也可以用 ``rosrun gesture_control_drone drone_control_node.py`` 单独跑）。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gesture_ros_node import main  # noqa: E402

if __name__ == "__main__":
    main()
