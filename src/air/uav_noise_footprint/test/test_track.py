#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞行剖面 core/track.py：斜爬升几何、匀速性、侧移与变体。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from uav_noise_footprint.core.exposure import sample_track
from uav_noise_footprint.core.track import Flight


@pytest.fixture
def flight():
    return Flight("UAV-1", [(0, 0), (1600, 0)], alt=30.0, t0=0.0,
                  speed=10.0, climb_angle=8.0)


def test_climb_run_geometry(flight):
    """爬升水平投影 = alt / tan(angle)，巡航段高度恒为 alt。"""
    assert flight.climb_run == pytest.approx(30.0 / math.tan(math.radians(8.0)))
    mid = flight.position_at((flight.start_time + flight.end_time) / 2)
    assert mid[2] == pytest.approx(30.0)
    # 中点在地面上应位于航路中点附近
    assert abs(mid[0] - 800.0) < 60.0


def test_ground_start_and_end(flight):
    p0 = flight.position_at(flight.start_time)
    p1 = flight.position_at(flight.end_time)
    assert p0[:2] == pytest.approx([0.0, 0.0])
    assert p0[2] == pytest.approx(0.0)
    assert p1[:2] == pytest.approx([1600.0, 0.0])
    assert p1[2] == pytest.approx(0.0)


def test_speed_constant_along_path(flight):
    """三段匀速：每段的速率都等于 speed。"""
    for seg in flight.segments:
        assert float(np.linalg.norm(seg.velocity)) == pytest.approx(
            10.0, rel=1e-6)


def test_altitude_profile_rises_then_falls(flight):
    zs = [flight.position_at(t)[2] for t in
          np.linspace(flight.start_time, flight.end_time, 50)]
    assert zs[0] == pytest.approx(0.0)
    assert max(zs) == pytest.approx(30.0)
    assert zs[-1] == pytest.approx(0.0)
    # 高度序列先升后降
    k = int(np.argmax(zs))
    assert zs[k] > zs[5] and zs[k] > zs[-5]


def test_with_climb_angle_changes_run(flight):
    steep = flight.with_climb_angle(12.0)
    assert steep.climb_run < flight.climb_run
    assert steep.climb_angle == pytest.approx(12.0)
    # 更陡 → 爬升段 3D 长度更短（alt/sin(angle)，贴地时间更少）
    assert steep.segments[0].length() < flight.segments[0].length()


def test_with_altitude(flight):
    hi = flight.with_altitude(60.0)
    assert hi.alt == pytest.approx(60.0)
    assert hi.climb_run > flight.climb_run


def test_lateral_offset_keeps_ends_and_shifts_mid(flight):
    off = flight.with_lateral_offset(120.0)
    assert off.path[0] == pytest.approx(flight.path[0])
    assert off.path[-1] == pytest.approx(flight.path[-1])
    # 巡航中段被横移约 120 m（2D 航路点级）
    mid_pts = off.path[len(off.path) // 2]
    assert abs(mid_pts[1] - 0.0) == pytest.approx(120.0, abs=5.0)
    # 偏移后 LAeq 场仍然可算（几何自洽）
    pos, dt = sample_track(off, 2.0)
    assert pos.shape[1] == 3 and dt > 0


def test_infeasible_altitude_raise_rejected():
    """航段容不下抬升后的爬升投影 → 构造必须报错。"""
    f = Flight("U", [(0, 0), (500, 0)], alt=30.0, speed=10.0, climb_angle=8.0)
    with pytest.raises(ValueError):
        f.with_altitude(90.0)          # run 639 m > 航段 500 m


def test_cruise_degenerate_rejected():
    """爬升+下降投影吃满航路 → 报错而不是倒飞。"""
    with pytest.raises(ValueError):
        Flight("U", [(0, 0), (300, 0)], alt=30.0, speed=10.0, climb_angle=8.0)


def test_roundtrip_dict(flight):
    d = flight.to_dict()
    g = Flight.from_dict(d)
    assert g.summary() == flight.summary()
