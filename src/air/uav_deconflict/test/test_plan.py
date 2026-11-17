#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航迹模型测试：分段构建、任意时刻位姿、三种解脱变体。"""
import math

import numpy as np
import pytest

from uav_deconflict.core.plan import Plan


def test_segments_cover_climb_cruise_descent():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, t0=5.0, speed=10.0,
             climb_speed=2.0)
    # 爬升 10 s + 平飞 10 s + 下降 10 s
    assert p.start_time == pytest.approx(5.0)
    assert p.end_time == pytest.approx(35.0)
    assert len(p.segments) == 3
    assert p.segments[0].p0[2] == pytest.approx(0.0)
    assert p.segments[0].p1[2] == pytest.approx(20.0)
    assert p.segments[1].p1[:2] == pytest.approx([100.0, 0.0])
    assert p.segments[2].p1[2] == pytest.approx(0.0)


def test_position_at_key_instants():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, t0=0.0, speed=10.0,
             climb_speed=2.0)
    assert p.position_at(5.0) == pytest.approx([0.0, 0.0, 10.0])
    assert p.position_at(15.0) == pytest.approx([50.0, 0.0, 20.0])
    assert p.position_at(25.0) == pytest.approx([100.0, 0.0, 10.0])
    # 未起飞 / 已落地
    assert p.position_at(-0.1) is None
    assert p.position_at(30.1) is None


def test_velocity_is_segmentwise():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, speed=10.0, climb_speed=2.0)
    assert p.velocity_at(5.0) == pytest.approx([0.0, 0.0, 2.0])
    assert p.velocity_at(15.0) == pytest.approx([10.0, 0.0, 0.0])
    assert p.velocity_at(25.0) == pytest.approx([0.0, 0.0, -2.0])
    assert p.velocity_at(99.0) is None


def test_hold_position_clamps_to_ground_and_endpoint():
    p = Plan("A", [(10, 20), (110, 20)], alt=20.0, t0=10.0, speed=10.0,
             climb_speed=2.0)
    assert p.hold_position(0.0) == pytest.approx([10.0, 20.0, 0.0])
    assert p.hold_position(1e6) == pytest.approx([110.0, 20.0, 0.0])


def test_ground_delay_shifts_everything():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, t0=3.0, speed=10.0,
             climb_speed=2.0)
    q = p.with_ground_delay(12.0)
    assert q.t0 == pytest.approx(15.0)
    assert q.start_time == pytest.approx(p.start_time + 12.0)
    assert q.end_time == pytest.approx(p.end_time + 12.0)
    assert q.position_at(15.0) == pytest.approx(p.position_at(3.0))


def test_altitude_change_keeps_horizontal_schedule():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, speed=10.0, climb_speed=2.0)
    q = p.with_altitude(50.0)
    # 爬升时间从 10 s 增到 25 s，但平飞段仍是 10 s
    assert q.end_time - p.end_time == pytest.approx(2 * (50 - 20) / 2.0)
    assert q.position_at(q.start_time + 25.0 + 5.0)[:2] == pytest.approx([50.0, 0.0])


def test_lateral_offset_is_smooth_and_reaches_peak():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, speed=10.0, climb_speed=2.0)
    q = p.with_lateral_offset(30.0, samples=8)
    # 起终点不动，中间插入 7 个采样点
    assert len(q.path) == 2 + 7
    assert q.path[0] == p.path[0] and q.path[-1] == p.path[-1]
    # 中点偏移应等于设定值（左法向为 +y）
    mid = q.path[4]
    assert mid[0] == pytest.approx(50.0)
    assert mid[1] == pytest.approx(30.0)
    # 相邻点横向增量应平滑：不允许一帧内出现接近全额的跳跃
    ys = [pt[1] for pt in q.path]
    steps = [abs(b - a) for a, b in zip(ys[:-1], ys[1:])]
    assert max(steps) < 0.5 * 30.0
    assert q.path_length_xy() > p.path_length_xy()


def test_lateral_offset_zero_is_identity():
    p = Plan("A", [(0, 0), (100, 0)], alt=20.0, speed=10.0)
    assert p.with_lateral_offset(0.0).path == p.path


def test_invalid_inputs_rejected():
    with pytest.raises(ValueError):
        Plan("A", [(0, 0)], alt=20.0)
    with pytest.raises(ValueError):
        Plan("A", [(0, 0), (1, 1)], alt=-1.0)
    with pytest.raises(ValueError):
        Plan("A", [(0, 0), (1, 1)], speed=0.0)
