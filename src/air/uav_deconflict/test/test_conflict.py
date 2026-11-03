#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""冲突检测测试：解析解与已知几何答案对照。"""
import math

import pytest

from uav_deconflict.core.conflict import (Separation, conflict_matrix, detect,
                                          pair_conflicts)
from uav_deconflict.core.plan import Plan


def head_on(gap_alt=0.0):
    a = Plan("A", [(-100, 0), (100, 0)], alt=30.0, speed=10.0, climb_speed=3.0)
    b = Plan("B", [(100, 0), (-100, 0)], alt=30.0 + gap_alt, speed=10.0,
             climb_speed=3.0)
    return a, b


def test_head_on_conflict_window_is_analytic():
    a, b = head_on()
    cons = pair_conflicts(a, b, Separation())
    assert len(cons) == 1
    c = cons[0]
    # 相对速度 20 m/s，水平距离 400-20t；|h|<50 → t ∈ (17.5, 22.5)
    assert c.t_start == pytest.approx(17.5, abs=0.05)
    assert c.t_end == pytest.approx(22.5, abs=0.05)
    assert c.min_h == pytest.approx(0.0, abs=0.05)
    assert c.min_v == pytest.approx(0.0, abs=0.05)


def test_vertical_separation_prevents_conflict():
    a, b = head_on(gap_alt=30.0)   # 30 m > 20 m 垂直间隔
    assert pair_conflicts(a, b, Separation()) == []


def test_parallel_routes_at_80m_are_safe():
    a = Plan("A", [(0, 0), (200, 0)], alt=40.0, speed=10.0)
    b = Plan("B", [(0, 80), (200, 80)], alt=40.0, speed=10.0)
    assert pair_conflicts(a, b, Separation()) == []


def test_parallel_routes_at_50m_are_tangent_and_safe():
    a = Plan("A", [(0, 0), (200, 0)], alt=40.0, speed=10.0)
    b = Plan("B", [(0, 50), (200, 50)], alt=40.0, speed=10.0)
    # 恰好等于间隔标准：按"小于"判定，切线不算侵入
    assert pair_conflicts(a, b, Separation()) == []


def test_parallel_routes_at_49m_conflict():
    a = Plan("A", [(0, 0), (200, 0)], alt=40.0, speed=10.0)
    b = Plan("B", [(0, 49), (200, 49)], alt=40.0, speed=10.0)
    cons = pair_conflicts(a, b, Separation())
    assert cons and cons[0].min_h == pytest.approx(49.0, abs=0.2)


def test_time_separation_removes_conflict():
    a = Plan("A", [(-100, 0), (100, 0)], alt=30.0, speed=10.0, climb_speed=3.0)
    b = Plan("B", [(100, 0), (-100, 0)], alt=30.0, speed=10.0, climb_speed=3.0,
             t0=100.0)   # 晚 100 s 起飞，A 已落地
    assert pair_conflicts(a, b, Separation()) == []


def test_crossing_conflict_has_positive_duration():
    a = Plan("A", [(-200, 0), (200, 0)], alt=50.0, speed=10.0)
    b = Plan("B", [(0, -200), (0, 200)], alt=50.0, speed=10.0)
    cons = pair_conflicts(a, b, Separation())
    assert len(cons) == 1
    assert cons[0].duration > 0
    assert cons[0].min_h < 1.0


def test_detect_is_symmetric_in_geometry():
    a, b = head_on()
    c1 = pair_conflicts(a, b, Separation())[0]
    c2 = pair_conflicts(b, a, Separation())[0]
    assert c1.t_start == pytest.approx(c2.t_start)
    assert c1.min_h == pytest.approx(c2.min_h)


def test_detect_returns_sorted_and_counts_pairs():
    a, b = head_on()
    c = Plan("C", [(0, -300), (0, 300)], alt=30.0, speed=10.0)
    cons = detect([a, b, c], Separation())
    assert cons == sorted(cons, key=lambda x: (x.t_start, x.a, x.b))
    mat = conflict_matrix([a, b, c], Separation())
    assert sum(mat.values()) == len(cons)
    assert ("A", "B") in mat


def test_single_plan_no_conflict():
    a = Plan("A", [(0, 0), (100, 0)], alt=30.0, speed=10.0)
    assert detect([a], Separation()) == []


def test_severity_bounds():
    a, b = head_on()
    c = pair_conflicts(a, b, Separation())[0]
    assert 0.0 <= c.severity() <= 1.0
    assert c.severity() == pytest.approx(1.0)


def test_invalid_separation_rejected():
    with pytest.raises(ValueError):
        Separation(horizontal=0.0)
