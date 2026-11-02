#!/usr/bin/env python3
"""
AprilTag 降落模块纯计算单元测试.
不依赖 ROS, 直接运行: python3 test_apriltag_landing.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from landing_math import pixel_to_ground, ST_SEARCH, ST_ALIGN, ST_DESCEND, ST_LANDED


def test_pixel_to_ground():
    fx, fy = 500.0, 500.0
    h = 5.0
    # 像素偏差 50px → 地面位移 0.5m
    gx, gy = pixel_to_ground(50.0, -20.0, h, fx, fy)
    assert abs(gx - 0.5) < 1e-6, f"gx={gx}"
    assert abs(gy + 0.2) < 1e-6, f"gy={gy}"
    print("[OK] pixel_to_ground 基本换算")


def test_pixel_to_ground_zero():
    gx, gy = pixel_to_ground(0.0, 0.0, 5.0, 500.0, 500.0)
    assert gx == 0.0 and gy == 0.0
    print("[OK] pixel_to_ground 零偏差")


def test_pixel_to_ground_scale_with_height():
    # 高度翻倍, 地面位移也翻倍
    gx1, _ = pixel_to_ground(50.0, 0.0, 5.0, 500.0, 500.0)
    gx2, _ = pixel_to_ground(50.0, 0.0, 10.0, 500.0, 500.0)
    assert abs(gx2 - 2 * gx1) < 1e-6
    print("[OK] pixel_to_ground 与高度成正比")


def test_state_constants():
    assert ST_SEARCH == "SEARCH"
    assert ST_ALIGN == "ALIGN"
    assert ST_DESCEND == "DESCEND"
    assert ST_LANDED == "LANDED"
    print("[OK] 状态机常量")


if __name__ == "__main__":
    test_pixel_to_ground()
    test_pixel_to_ground_zero()
    test_pixel_to_ground_scale_with_height()
    test_state_constants()
    print("\n全部测试通过")
