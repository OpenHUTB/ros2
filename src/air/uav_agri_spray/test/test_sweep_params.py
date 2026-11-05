#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Parameter sweep: grid shape, recommendation policy, monotonic duration."""
import pytest

from uav_agri_spray.core.planner import SprayConfig
from uav_agri_spray.core.sweep import (MIN_COVERAGE, SWEEP_OVERLAPS,
                                       SWEEP_SPEEDS, recommend, sweep_case)
from uav_agri_spray.core.scenarios import get_case


@pytest.fixture(scope="module")
def rect_points():
    return sweep_case(get_case("case_field_rect"), SprayConfig())


@pytest.fixture(scope="module")
def obs_points():
    return sweep_case(get_case("case_field_obstacle"), SprayConfig())


def test_sweep_grid_shape(rect_points):
    assert len(rect_points) == len(SWEEP_SPEEDS) * len(SWEEP_OVERLAPS)
    combos = {(p.speed, p.overlap) for p in rect_points}
    assert combos == {(s, o) for s in SWEEP_SPEEDS for o in SWEEP_OVERLAPS}


def test_recommendation_meets_threshold(rect_points):
    rec = recommend(rect_points)
    assert rec.coverage_ratio >= MIN_COVERAGE
    # 达标组合里时长必须最短
    qualified = [p for p in rect_points if p.coverage_ratio >= MIN_COVERAGE]
    assert rec.duration_s == min(p.duration_s for p in qualified)


def test_recommendation_flagged_in_table(rect_points):
    flagged = [p for p in rect_points if p.recommended]
    assert len(flagged) == 1
    assert flagged[0].speed == recommend(rect_points).speed


def test_duration_decreases_with_speed(rect_points):
    for ov in SWEEP_OVERLAPS:
        durs = [p.duration_s for p in rect_points
                if p.overlap == ov]
        assert durs[0] > durs[1] > durs[2]  # 3 > 4 > 5 m/s


def test_obstacle_case_falls_back_to_best_coverage(obs_points):
    best_cov = max(p.coverage_ratio for p in obs_points)
    assert best_cov < MIN_COVERAGE  # 没有组合达标
    rec = recommend(obs_points)
    assert rec.coverage_ratio == best_cov


def test_longer_path_at_higher_overlap(rect_points):
    lens = {p.overlap: p.total_length_m for p in rect_points
            if p.speed == 4.0}
    assert lens[0.1] < lens[0.2] < lens[0.3]
