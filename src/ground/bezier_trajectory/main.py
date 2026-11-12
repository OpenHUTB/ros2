#!/usr/bin/env python3
"""根目录入口：转发到 ROS2 包内的 main。"""
import sys
import os

# 让 bezier_trajectory 作为包被导入（支持相对导入）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bezier_trajectory.main import main

if __name__ == "__main__":
    main()
