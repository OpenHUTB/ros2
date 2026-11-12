#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_l1_altitude_stabilization 包根入口：转发到 scripts/uav_l1_altitude_stabilization.py。"""
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "scripts", "uav_l1_altitude_stabilization.py")

if __name__ == "__main__":
    sys.path.insert(0, os.path.join(HERE, "scripts"))
    runpy.run_path(TARGET, run_name="__main__")
