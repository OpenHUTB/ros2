#!/usr/bin/env python3
"""模块入口：``python main.py demo`` 即可跑完整演示。

ROS 2 启动时（``ros2 launch brake_thermal_fade main.launch.py``）不带子命令，
命令行参数会原样交给 ``ros_node``，分派逻辑见 ``brake_thermal_fade/cli.py``。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from brake_thermal_fade.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
