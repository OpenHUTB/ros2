#!/usr/bin/env python3
"""Gazebo 差速驱动小车 —— 模块入口（课程 main.* 约定）。

实际实现在 ``gazebo_diff_drive_car/main.py``，本文件仅作为可直接执行的
入口，便于 ``python3 main.py`` 或 ``./main.sh`` 使用。

用法：
    python3 main.py                                  # 键盘遥控（默认）
    python3 main.py --ros-args -p mode:=laser        # 激光雷达避障
    python3 main.py --ros-args -p mode:=camera       # 深度相机避障
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from gazebo_diff_drive_car.main import main  # noqa: E402


if __name__ == '__main__':
    main()
