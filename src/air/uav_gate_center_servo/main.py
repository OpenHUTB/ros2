#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_gate_center_servo 入口。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from uav_gate_center_servo.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
