#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_control_bridge`` 模块统一入口（main.* 约定）。

作业约定：每个模块的入口必须是 ``main.`` 开头，且 ``roslaunch`` 与 ``rosrun``
两种方式都能直接跑起来整个模块::

    roslaunch gesture_control_bridge main.launch          # 作业要求的 launch 启动
    rosrun   gesture_control_bridge main.py               # 只跑"手势指令 -> 桥接层"节点

入口本身不含逻辑，只把节点逻辑转发过去，保证两种启动方式行为一致。
手势**感知**节点在 ``gesture_control_drone`` 包里，由 launch 以
``<node pkg="gesture_control_drone" type="main.py" .../>`` 拉起（也可单独 rosrun）。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gesture_bridge_node import main  # noqa: E402

if __name__ == "__main__":
    main()
