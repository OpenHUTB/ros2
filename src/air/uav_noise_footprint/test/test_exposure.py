#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""暴露评估 core/exposure.py：LAeq 合成、足迹栅格与 marching squares。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from uav_noise_footprint.core.acoustic import DEFAULT
from uav_noise_footprint.core.exposure import (SensitivePoint,
                                               contour_segments,
                                               contour_summary,
                                               current_levels, exceedances,
                                               grid_footprint, laeq_at_points,
                                               laeq_from_samples, objective,
                                               sample_track)
from uav_noise_footprint.core.track import Flight


@pytest.fixture
def flight():
    return Flight("UAV-1", [(0, 0), (1600, 0)], alt=30.0, t0=0.0,
                  speed=10.0, climb_angle=8.0)


def test_sample_track_step_and_shape(flight):
    pos, dt = sample_track(flight, ds=2.0)
    assert pos.ndim == 2 and pos.shape[1] == 3
    # 步长 ≈ 2 m（总弧长 / 样本数）
    step = flight.path_length_3d() / (len(pos) - 1)
    assert step == pytest.approx(2.0, abs=0.05)
    assert dt == pytest.approx(2.0 / 10.0, abs=0.01)


def test_laeq_bounded_by_closest_approach(flight):
    """LAeq 不会超过最近逼近点的 Lp，也不应低于它 20 dB 以上。"""
    pos, dt = sample_track(flight, 2.0)
    lmax = DEFAULT.lp_from(pos[np.argmin(
        np.linalg.norm(pos - [800.0, 40.0, 0.0], axis=1))], (800.0, 40.0, 0.0))
    laeq = laeq_from_samples(pos, dt, (800.0, 40.0))
    assert laeq <= lmax + 0.5
    assert laeq > lmax - 20.0


def test_laeq_symmetric_about_track(flight):
    pts = [SensitivePoint("L", 800.0, -40.0), SensitivePoint("R", 800.0, 40.0)]
    v = laeq_at_points([flight], pts, 2.0)
    assert v["L"] == pytest.approx(v["R"], abs=0.05)


def test_repeat_traffic_density_adds_10log10(flight):
    pts = [SensitivePoint("p", 800.0, 40.0)]
    one = laeq_at_points([flight], pts, 2.0, repeat=1, t_obs=3600.0)["p"]
    ten = laeq_at_points([flight], pts, 2.0, repeat=10, t_obs=3600.0)["p"]
    # 同一观测期内重复 10 次 → +10 dB
    assert ten - one == pytest.approx(10.0, abs=0.1)


def test_objective_weight_dominates(flight):
    quiet = [SensitivePoint("a", 800.0, 400.0, weight=1.0)]
    loud = [SensitivePoint("b", 800.0, 10.0, weight=1.0)]
    assert objective([flight], loud) > objective([flight], quiet)


def test_grid_footprint_shape_and_contrast(flight):
    grid, ext = grid_footprint([flight], n=200, ds=2.0)
    assert grid.shape == (200, 200)
    xmin, xmax, ymin, ymax = ext
    ix = int((800.0 - xmin) / (xmax - xmin) * 199)
    iy = int((0.0 - ymin) / (ymax - ymin) * 199)
    corner = grid[0, 0]
    assert grid[iy, ix] > corner + 5.0
    # 数值上必须与单点 LAeq 一致（同一套能量积分）
    pts = [SensitivePoint("p", 800.0, 0.0)]
    laeq = laeq_at_points([flight], pts, 2.0)["p"]
    assert grid[iy, ix] == pytest.approx(laeq, abs=0.1)


def test_contour_segments_linear_field():
    """已知线性场 grid[j,i]=x：level=0.5 的等值线应竖在 x=0.5 处。"""
    n = 50
    xs = np.linspace(0.0, 1.0, n)
    grid = np.tile(xs, (n, 1))
    ext = (0.0, 1.0, 0.0, 1.0)
    segs = contour_segments(grid, 0.5, ext)
    assert len(segs) == n - 1
    for a, b in segs:
        for p in (a, b):
            assert p[0] == pytest.approx(0.5, abs=0.02)


def test_contour_summary_areas_monotonic(flight):
    grid, ext = grid_footprint([flight], n=200, ds=2.0)
    rows = contour_summary(grid, ext, (45.0, 55.0, 65.0))
    areas = [r["area_m2"] for r in rows]
    assert areas[0] > areas[1] > areas[2]
    assert rows[2]["level"] == 65.0


def test_current_levels_energy_sum():
    """节点判定：两机在敏感点正上方 30 m → 能量叠加 +3 dB。"""
    pts = [SensitivePoint("p", 0.0, 0.0)]
    one = current_levels({"A": (0.0, 0.0, 30.0)}, pts)["p"]
    two = current_levels({"A": (0.0, 0.0, 30.0), "B": (0.0, 0.0, 30.0)},
                         pts)["p"]
    assert two - one == pytest.approx(3.0103, abs=1e-2)


def test_exceedances_flag_only_over_limit():
    pts = [SensitivePoint("p", 0.0, 0.0, "residential"),   # 限值 60
           SensitivePoint("s", 500.0, 0.0, "school")]      # 限值 55
    levels = current_levels({"A": (0.0, 0.0, 25.0),        # p 处 ~65.6 dB
                             "B": (500.0, 0.0, 40.0)},     # s 处 ~56 dB
                            pts)
    alerts = exceedances(levels, pts)
    names = {a["point"] for a in alerts}
    assert names == {"p", "s"}
    by = {a["point"]: a for a in alerts}
    assert by["p"]["exceed"] == pytest.approx(levels["p"] - 60.0, abs=0.1)
    # 都降到限值以下 → 无告警
    levels2 = current_levels({"A": (0.0, 0.0, 200.0), "B": (500.0, 0.0, 200.0)},
                             pts)
    assert exceedances(levels2, pts) == []
