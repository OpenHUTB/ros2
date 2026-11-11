#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯追踪控制核心测试（不依赖 ROS）。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from diff_drive_astar_odom_nav.control import (  # noqa: E402
    lookahead_point, nearest_index, pure_pursuit_step, wrap_angle)


def test_nearest_index_monotonic():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert nearest_index(path, (0.1, 0.0), 0)[0] == 0
    assert nearest_index(path, (1.9, 0.0), 1)[0] == 2


def test_lookahead_point_advances():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    point, idx = lookahead_point(path, 0, 1.5)
    assert abs(point[0] - 1.5) < 1e-9
    assert idx == 1


def test_straight_tracking_has_zero_angular_command():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    step = pure_pursuit_step(path, (0.0, 0.0, 0.0), 0, lookahead=0.5)
    assert step["v"] > 0.0
    assert abs(step["w"]) < 1e-6


def test_offset_requires_left_turn():
    path = [(0.0, 0.0), (3.0, 0.0)]
    step = pure_pursuit_step(path, (0.0, -0.5, 0.0), 0, lookahead=0.6)
    assert step["w"] > 0.0  # 目标在左侧（+y），需要左转（w>0）


def test_reached_within_tolerance():
    path = [(0.0, 0.0), (3.0, 0.0)]
    step = pure_pursuit_step(path, (2.95, 0.0, 0.0), 1, goal_tolerance=0.12)
    assert step["reached"]
    assert step["v"] == 0.0 and step["w"] == 0.0


def test_wrap_angle():
    assert abs(abs(wrap_angle(3.0 * math.pi)) - math.pi) < 1e-9
