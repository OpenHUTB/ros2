#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Edge cases: far obstacles, thin fields, degenerate inputs, multi-obstacle."""
import numpy as np
import pytest

from uav_agri_spray.core.geometry import CircleObstacle
from uav_agri_spray.core.planner import SprayConfig, plan_field


def _rect(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def test_far_obstacle_changes_nothing():
    far = CircleObstacle(500.0, 500.0, 2.0)
    p0 = plan_field((_rect(0, 0, 50, 36),), (), SprayConfig())
    p1 = plan_field((_rect(0, 0, 50, 36),), (far,), SprayConfig())
    assert p0.total_length == pytest.approx(p1.total_length, rel=1e-6)


def test_obstacle_on_scanline_increases_length():
    obs = CircleObstacle(25.0, 18.0, 0.8)
    p0 = plan_field((_rect(0, 0, 50, 36),), (), SprayConfig())
    p1 = plan_field((_rect(0, 0, 50, 36),), (obs,), SprayConfig())
    assert p1.total_length > p0.total_length


def test_thin_field_single_line():
    p = plan_field((_rect(0, 0, 10, 1.5),), (), SprayConfig())
    assert len(p.lines) == 1


def test_degenerate_polygon_degrades_gracefully():
    # 两条共线顶点构成的退化"多边形"：无扫描区间，应返回空路径而不是崩溃
    p = plan_field((np.array([[0.0, 0.0], [10.0, 0.0]]),), (), SprayConfig())
    assert len(p.lines) == 0
    assert p.total_length == 0.0


def test_two_obstacles_on_same_line_both_detoured():
    obs = (CircleObstacle(15.0, 18.0, 0.8), CircleObstacle(35.0, 18.0, 0.8))
    p = plan_field((_rect(0, 0, 50, 36),), obs, SprayConfig())
    arcs_with = [ln for ln in p.lines if ln.arcs]
    assert len(arcs_with) >= 2  # 至少两条扫描线各带绕行弧
    R = 0.8 + SprayConfig().obstacle_margin
    for ln in p.lines:
        for arc in ln.arcs:
            for o in obs:
                # 每条弧只允许贴近自己对应的障碍，且都保持安全间隙
                d = np.linalg.norm(arc - o.center, axis=1)
                if d.min() < R * 2:
                    assert d.min() >= R - 0.05


def test_zero_overlap_guarded():
    with pytest.raises(ValueError):
        plan_field((_rect(0, 0, 50, 36),), (), SprayConfig(overlap=1.0))
