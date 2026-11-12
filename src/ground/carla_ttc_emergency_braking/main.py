#!/usr/bin/env python3
"""Convenience entry: python main.py ...  (equivalent to `ros2 run carla_ttc_emergency_braking carla_ttc_emergency_braking`)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_ttc_emergency_braking.app import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())