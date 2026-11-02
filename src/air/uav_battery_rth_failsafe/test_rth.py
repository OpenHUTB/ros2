#!/usr/bin/env python3
"""电池 RTH 模块纯计算单元测试, 不依赖 ROS."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'uav_battery_rth_failsafe'))
from rth_math import decide_state, dist_3d, ST_NORMAL, ST_WARNING, ST_RETURN_HOME, ST_EMERGENCY_LAND


def test_normal():
    s = decide_state(80, 25, 15, 8, 10, 0.15)
    assert s == ST_NORMAL
    print("[OK] 电量充足 → NORMAL")


def test_warning():
    s = decide_state(20, 25, 15, 8, 5, 0.15)
    assert s == ST_WARNING
    print("[OK] 预警电量 → WARNING")


def test_rth():
    s = decide_state(12, 25, 15, 8, 100, 0.15)
    assert s == ST_RETURN_HOME
    print("[OK] 返航电量 → RETURN_HOME")


def test_emergency():
    s = decide_state(5, 25, 15, 8, 100, 0.15)
    assert s == ST_EMERGENCY_LAND
    print("[OK] 迫降电量 → EMERGENCY_LAND")


def test_warning_but_insufficient():
    # 电量20%但返航需要20% (距离远), 直接返航
    s = decide_state(20, 25, 15, 8, 134, 0.15)
    assert s == ST_RETURN_HOME
    print("[OK] 预警但返航余量不足 → 立即 RETURN_HOME")


def test_dist_3d():
    assert abs(dist_3d((0,0,0), (3,4,0)) - 5.0) < 1e-6
    print("[OK] dist_3d")


if __name__ == "__main__":
    test_normal()
    test_warning()
    test_rth()
    test_emergency()
    test_warning_but_insufficient()
    test_dist_3d()
    print("\n全部测试通过")
