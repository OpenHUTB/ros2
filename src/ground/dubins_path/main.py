#!/usr/bin/env python3
"""根目录入口：转发到 ROS2 包内的 main。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dubins_path.main import main

if __name__ == "__main__":
    main()
