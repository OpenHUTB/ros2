#!/usr/bin/env python3
"""Convenience entry: python main.py ...  (equivalent to `ros2 run airsim_multirotor_waypoint_nav airsim_multirotor_waypoint_nav`)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from airsim_multirotor_waypoint_nav.app import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())