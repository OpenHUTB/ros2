#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三段航迹单元测试：进场 60 m 进近 / 离场 60 m 转巡航。"""
import numpy as np
import pytest

from uav_vertiport_queue.core.track import (GATE_ALT, Track, arrival_track,
                                            departure_track)


def test_departure_gate_then_cruise():
    tr = departure_track("A", (0, 0), (300, 0), t_takeoff=10.0,
                         cruise_alt=100.0, gate_alt=60.0,
                         climb_speed=2.5, speed=8.0)
    assert tr.kind == "departure"
    assert tr.start_time == pytest.approx(10.0)
    # 第一段结束：起飞位上方 60 m 过渡高度
    p1 = tr.segments[0].p1
    assert p1[2] == pytest.approx(GATE_ALT)
    assert p1[0] == pytest.approx(0.0)
    # 第二段结束：爬到巡航高度
    assert tr.segments[1].p1[2] == pytest.approx(100.0)
    # 终点在离场航路点、巡航高度
    end = tr.segments[-1].p1
    assert end[0] == pytest.approx(300.0) and end[2] == pytest.approx(100.0)
    # 计划离地时刻 = 航迹起点
    assert tr.op_time == pytest.approx(10.0)


def test_arrival_gate_then_touchdown():
    tr = arrival_track("B", (-300, 80), (0, 80), t_touchdown=200.0,
                       cruise_alt=100.0, gate_alt=60.0,
                       speed=8.0, descent_speed=2.0)
    assert tr.kind == "arrival"
    # 终点：降落位地面，时刻 = 计划接地时刻
    end = tr.segments[-1].p1
    assert end[2] == pytest.approx(0.0)
    assert tr.end_time == pytest.approx(200.0)
    assert tr.op_time == pytest.approx(200.0)
    # 第一段结束：定位点上方从巡航高度降到 60 m
    p1 = tr.segments[0].p1
    assert p1[2] == pytest.approx(GATE_ALT)
    assert p1[0] == pytest.approx(-300.0)
    # 进近直线段保持 60 m
    for s in tr.segments[1:2]:
        assert s.p0[2] == pytest.approx(60.0)
        assert s.p1[2] == pytest.approx(60.0)
    # 起始点在巡航高度
    assert tr.segments[0].p0[2] == pytest.approx(100.0)


def test_position_at_endpoints_and_hold():
    tr = departure_track("A", (0, 0), (300, 0), 0.0)
    assert np.allclose(tr.position_at(tr.start_time), tr.segments[0].p0)
    assert np.allclose(tr.position_at(tr.end_time), tr.segments[-1].p1)
    # 未起飞/结束后保持在起终点
    assert np.allclose(tr.hold_position(-10.0), tr.segments[0].p0)
    assert np.allclose(tr.hold_position(tr.end_time + 10.0),
                       tr.segments[-1].p1)
    assert tr.position_at(-10.0) is None


def test_uniform_velocity_on_segment():
    tr = arrival_track("B", (-300, 80), (0, 80), 100.0)
    seg = tr.segments[1]
    t0, t1 = seg.t0, seg.t1
    v = seg.velocity
    pm = seg.position((t0 + t1) / 2.0)
    assert np.allclose(pm, seg.p0 + v * (t1 - t0) / 2.0)
    # 匀速：任意两时刻速度一致
    assert np.allclose(tr.velocity_at(t0 + 0.1), tr.velocity_at(t1 - 0.1))


def test_track_rejects_bad_input():
    with pytest.raises(ValueError):
        Track("X", "taxi", 0.0, segments=[])
    with pytest.raises(ValueError):
        departure_track("X", (0, 0), (300, 0), 0.0, cruise_alt=50.0)
    with pytest.raises(ValueError):
        arrival_track("X", (-300, 80), (0, 80), 0.0, cruise_alt=30.0)
