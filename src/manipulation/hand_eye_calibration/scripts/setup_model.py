#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口：下载 Panda 网格资源。"""
import os
import sys

# 以 `python3 scripts/setup_model.py` 运行时 sys.path[0] 是 scripts/ 目录，
# 需把模块根目录加入 sys.path 才能导入 hand_eye_calibration 包。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hand_eye_calibration.setup_model import main  # noqa: E402


if __name__ == "__main__":
    main()
