#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景定义（core.scenarios）单元测试。"""
import math

import numpy as np
import pytest

from uav_right_of_way.core.scenarios import (FlightPlan, SCENARIOS,
                                             get_scenario)


def test_three_scenarios_four_aircraft():
    assert set(SCENARIOS) == {"headon", "cross", "overtake"}
    for name in SCENARIOS:
        sc = get_scenario(name)
        assert len(sc.plans) == 4
        assert len({p.uid for p in sc.plans}) == 4


def test_plan_phases_and_queries():
    p = FlightPlan("T", (0, 0), (100, 0), alt=10.0, speed=5.0, climb_speed=2.5)
    assert p.cruise_t0 == pytest.approx(4.0)          # 爬升 10/2.5
    assert p.cruise_t1 == pytest.approx(4.0 + 20.0)   # 巡航 100/5
    assert p.end_time == pytest.approx(28.0)          # 再加下降 10/2.5
    assert p.position_at(0.0)[2] == pytest.approx(0.0)
    assert p.position_at(2.0)[2] == pytest.approx(5.0)
    mid = p.position_at(14.0)
    assert mid[0] == pytest.approx(50.0)
    assert mid[2] == pytest.approx(10.0)
    assert p.position_at(24.0)[2] == pytest.approx(10.0)   # 下降起点
    assert p.position_at(26.0)[2] == pytest.approx(5.0)
    assert p.position_at(28.0)[2] == pytest.approx(0.0)
    assert p.in_cruise(10.0) and not p.in_cruise(2.0)
    v = p.velocity_at(10.0)
    assert v[0] == pytest.approx(5.0) and v[2] == pytest.approx(0.0)


def test_cross_track_sign():
    p = FlightPlan("T", (0, 0), (100, 0), alt=10.0, speed=5.0)
    assert p.cross_track([50.0, 20.0]) == pytest.approx(20.0)   # 左侧为正
    assert p.cross_track([50.0, -20.0]) == pytest.approx(-20.0)
    assert p.along_track([30.0, 7.0]) == pytest.approx(30.0)


def test_headon_geometry():
    sc = get_scenario("headon")
    for a_uid, b_uid in (("UAV-1", "UAV-2"), ("UAV-3", "UAV-4")):
        a = next(p for p in sc.plans if p.uid == a_uid)
        b = next(p for p in sc.plans if p.uid == b_uid)
        assert a.alt == b.alt                                   # 同高度
        hdg_diff = abs(math.atan2(math.sin(b.cruise_heading - a.cruise_heading),
                                  math.cos(b.cruise_heading - a.cruise_heading)))
        assert hdg_diff == pytest.approx(math.pi, abs=1e-6)     # 对头
        # 同一条走廊
        assert a.start[1] == b.start[1]


def test_cross_meets_at_center():
    sc = get_scenario("cross")
    for p in sc.plans:
        assert p.speed == 8.0 and p.alt == 15.0
        # 巡航中点都过空域中心
        t_mid = 0.5 * (p.cruise_t0 + p.cruise_t1)
        pos = p.position_at(t_mid)
        assert abs(pos[0]) < 1e-6 and abs(pos[1]) < 1e-6


def test_overtake_rear_faster_same_lane():
    sc = get_scenario("overtake")
    u1 = next(p for p in sc.plans if p.uid == "UAV-1")
    u2 = next(p for p in sc.plans if p.uid == "UAV-2")
    assert u2.speed > u1.speed
    assert u1.start[1] == u2.start[1] == u1.end[1]              # 同航路
    assert u2.start[0] < u1.start[0]                            # 后机更靠后
    assert u2.cruise_heading == u1.cruise_heading


def test_plan_validation():
    with pytest.raises(ValueError):
        FlightPlan("X", (0, 0), (0, 0))
    with pytest.raises(ValueError):
        FlightPlan("X", (0, 0), (100, 0), speed=0.0)
