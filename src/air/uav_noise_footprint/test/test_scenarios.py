#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景注册与基线超标性：场景必须有真实的降噪动机。"""
from __future__ import annotations

import math

import pytest

from uav_noise_footprint.core.exposure import laeq_at_points
from uav_noise_footprint.core.scenarios import OBS_PERIOD, SCENARIOS, get_scenario


def test_three_scenarios_registered():
    assert set(SCENARIOS) == {"residential", "school", "mixed"}


def test_all_flights_geometrically_valid():
    for name in SCENARIOS:
        sc = get_scenario(name)
        assert sc.flights and sc.points
        for f in sc.flights:
            assert f.end_time > f.start_time >= 0
            assert f.path_length_ground() > 2.0 * f.climb_run


def test_school_baseline_exceeds_limit():
    """school 场景的基线必须真实超标（否则降噪没有动机）。"""
    sc = get_scenario("school")
    laeq = laeq_at_points(sc.flights, sc.points, 2.0,
                          repeat=sc.events_per_hour, t_obs=OBS_PERIOD)
    school = next(p for p in sc.points if p.name == "实验小学")
    assert laeq["实验小学"] > school.limit


def test_mixed_school_baseline_exceeds_limit():
    sc = get_scenario("mixed")
    laeq = laeq_at_points(sc.flights, sc.points, 2.0,
                          repeat=sc.events_per_hour, t_obs=OBS_PERIOD)
    school = next(p for p in sc.points if p.name == "中学")
    assert laeq["中学"] > school.limit


def test_unknown_scenario_rejected():
    with pytest.raises(KeyError):
        get_scenario("nope")
