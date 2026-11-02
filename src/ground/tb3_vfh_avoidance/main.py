#!/usr/bin/env python3
"""一键启动入口：直接运行本文件即启动 VFH 避障节点。

    python main.py
    python main.py --ros-args -p scan_topic:=/scan -p exit_after:=30.0
"""

import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tb3_vfh_avoidance.avoid_node import main     # noqa: E402

if __name__ == '__main__':
    main()
