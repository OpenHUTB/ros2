#!/usr/bin/env python3
"""Convenience entry: python main.py ...  (equivalent to `ros2 run carla_lidar_obstacle_detection carla_lidar_obstacle_detection`)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_lidar_obstacle_detection.app import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())