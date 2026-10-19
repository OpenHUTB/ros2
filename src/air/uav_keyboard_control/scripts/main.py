#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制模块统一入口（main.py）.

作业要求每模块入口为 main.*；这里复用 keyboard_control 的节点逻辑，
使 `roslaunch uav_keyboard_control main.launch` 与 `rosrun uav_keyboard_control main.py`
都能直接启动键盘控制。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from keyboard_control import main  # noqa: E402

if __name__ == "__main__":
    main()
