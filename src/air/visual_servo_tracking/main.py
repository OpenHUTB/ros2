#!/usr/bin/env python3
"""无人机视觉伺服目标跟踪 — 启动入口。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from visual_servo_tracking import run_demo

if __name__ == "__main__":
    run_demo()
