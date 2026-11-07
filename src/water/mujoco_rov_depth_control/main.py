#!/usr/bin/env python3
"""Convenience entry: python main.py ...  (equivalent to `ros2 run mujoco_rov_depth_control mujoco_rov_depth_control`)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mujoco_rov_depth_control.app import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())