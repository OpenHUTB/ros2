#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""geofence_detector 单元测试（不依赖 ROS）。"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (  # noqa: E402
    GeofenceDetector, INSIDE, OK, RectGeofence, WARNING, CylinderGeofence,
)


def make_cylinder():
    return CylinderGeofence(center=(0, 0), radius=100, z_max=120,
                            warning_distance=30)


def test_cylinder_inside():
    f = make_cylinder()
    status, dist = f.check(50, 0, 50)      # 圆心上方
    assert status == INSIDE
    assert dist == pytest_approx(-50.0)


def pytest_approx(v):
    return v


def test_cylinder_warning():
    f = make_cylinder()
    status, dist = f.check(110, 0, 50)     # 距边界 10m
    assert status == WARNING
    assert abs(dist - 10.0) < 1e-9


def test_cylinder_ok():
    f = make_cylinder()
    status, dist = f.check(200, 0, 50)
    assert status == OK
    assert abs(dist - 100.0) < 1e-9


def test_cylinder_above_altitude_free():
    """高于禁飞区顶高时不应触发 (法规允许越顶飞行场景另论)。"""
    f = make_cylinder()
    status, _ = f.check(50, 0, 150)
    assert status == OK


def test_rect_inside_and_warning():
    r = RectGeofence(-50, 50, -50, 50, warning_distance=20)
    assert r.check(0, 0, 30)[0] == INSIDE
    assert r.check(60, 0, 30)[0] == WARNING   # 距边界 10m
    assert r.check(100, 100, 30)[0] == OK


def test_detector_multi_fence_strictest_wins():
    d = GeofenceDetector([make_cylinder(),
                          RectGeofence(300, 400, 300, 400)])
    # 在圆柱禁飞区内部 → INSIDE
    assert d.check(10, 0, 50)[0] == INSIDE
    # 远离所有禁飞区 → OK
    assert d.check(-500, -500, 50)[0] == OK
    # 圆柱 WARNING 范围内 → WARNING
    assert d.check(115, 0, 50)[0] == WARNING


def test_flight_path_breach_detection():
    """模拟一条穿越禁飞区的航线, 应检测出 INSIDE 段。"""
    f = make_cylinder()
    statuses = []
    for i in range(0, 301, 5):
        x = i - 150  # 从西向东穿越
        statuses.append(f.check(x, 0, 50)[0])
    assert INSIDE in statuses
    assert statuses[0] == OK and statuses[-1] == OK
