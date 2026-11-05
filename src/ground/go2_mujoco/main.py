#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""源码树直跑入口：在模块根目录执行 `python3 main.py`。

仿真实现位于 src/go2_mujoco/main.py；本文件只把内层 src 目录加入 sys.path
后转发到包入口，使 README 与教程中的 `cd <模块根> && python3 main.py` 可直接使用。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

from go2_mujoco.main import main  # noqa: E402

if __name__ == "__main__":
    main()
