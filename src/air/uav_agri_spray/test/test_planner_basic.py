#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Coverage planner basics: direction, spacing, length, turns, detours."""
import numpy as np
import pytest

from uav_agri_spray.core.geometry import CircleObstacle, min_width_direction
from uav_agri_spray.core.planner import (ScanLine, SprayConfig, plan_convex,
                                         plan_field, progress_state)

RECT = np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 36.0], [0.0, 36.0]])
L_SUBS = (np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 16.0], [0.0, 16.0]]),
          np.array([[0.0, 16.0], [24.0, 16.0], [24.0, 40.0], [0.0, 40.0]]))


@pytest.fixture(scope="module")
def rect_path():
    return plan_field((RECT,), (), SprayConfig())


def test_min_width_direction_rect():
    d, width = min_width_direction(RECT)
    assert width == pytest.approx(36.0)
    assert abs(d[1]) < 1e-9  # 沿长边方向


def test_scanline_count_matches_spacing(rect_path):
    cfg = SprayConfig()
    assert len(rect_path.lines) == round(36.0 / cfg.spacing)
    assert len(rect_path.lines) == 15


def test_path_length_close_to_area_over_spacing(rect_path):
    # 面积/有效间距给出主航程；转入弧额外增加，应在 15% 以内
    s_eff = 36.0 / len(rect_path.lines)
    main_len = RECT[0, 0] * 0 + 50.0 * 15  # 15 条 50 m 扫描线
    assert rect_path.total_length == pytest.approx(main_len, rel=0.15)


def test_scanlines_inside_field(rect_path):
    s_eff = 36.0 / len(rect_path.lines)
    tol = s_eff / 2.0 + 1e-6  # 半圆转入弧允许外凸半个间距
    for ln in rect_path.lines:
        for p in ln.points:
            assert -tol <= p[0] <= 50.0 + tol
            assert -tol <= p[1] <= 36.0 + tol


def test_duration_is_length_over_speed(rect_path):
    cfg = SprayConfig()
    assert rect_path.duration == pytest.approx(
        rect_path.total_length / cfg.speed, rel=1e-6)


def test_semicircle_turn_geometry(rect_path):
    s_eff = 36.0 / len(rect_path.lines)
    ln0, ln1 = rect_path.lines[0], rect_path.lines[1]
    p_end = ln0.points[-1]
    p_start = ln1.turn_arc[-1]  # 转入弧末端 = 第 2 行起点
    # 相邻行同一侧端点间距 = 有效间距
    assert np.linalg.norm(p_end - p_start) == pytest.approx(s_eff, rel=1e-6)
    m = 0.5 * (p_end + p_start)
    radii = np.linalg.norm(ln1.turn_arc - m, axis=1)
    assert np.allclose(radii, s_eff / 2.0, atol=1e-6)


def test_detour_clears_obstacle():
    obs = CircleObstacle(25.0, 18.0, 0.8)
    cfg = SprayConfig()
    lines = plan_convex(RECT, [obs], cfg)
    R = obs.radius + cfg.obstacle_margin
    worst = np.inf
    for ln in lines:
        for arc in ln.arcs:
            dd = np.linalg.norm(arc - obs.center, axis=1)
            worst = min(worst, float(dd.min()))
    assert worst >= R - 0.05
    # 至少一条扫描线真的产生了绕行弧
    assert any(ln.arcs for ln in lines)


def test_lshape_two_subregions():
    path = plan_field(L_SUBS, (), SprayConfig())
    assert path.subregions == 2
    ids = [ln.line_id for ln in path.lines]
    assert ids == list(range(len(ids)))


def test_progress_state_milestone(rect_path):
    mid = rect_path.lines[5].points[len(rect_path.lines[5].points) // 2]
    st = progress_state(rect_path, mid)
    assert st["line_id"] == 5
    assert 0.0 < st["area_percent"] < 100.0
    head = progress_state(rect_path, rect_path.lines[0].points[0])
    assert head["line_id"] == 0
    assert head["area_percent"] == pytest.approx(0.0, abs=1.0)
