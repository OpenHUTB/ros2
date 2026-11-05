#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop smoke test: small field, two lines, under ~10 s wall."""
import numpy as np
import pytest

from uav_agri_spray.core.flyer import fly_first_lines
from uav_agri_spray.core.planner import SprayConfig, plan_field

pytest.importorskip("mujoco")

FIELD = np.array([[0.0, 0.0], [12.0, 0.0], [12.0, 8.0], [0.0, 8.0]])


@pytest.fixture(scope="module")
def flight():
    cfg = SprayConfig(overlap=0.2, speed=4.0)
    path = plan_field((FIELD,), (), cfg)
    return path, fly_first_lines(path, 2, cfg, FIELD)


def test_flight_completes_quickly(flight):
    _path, res = flight
    assert res.flown_lines == 2
    assert res.wall_time_s < 20.0
    assert res.flown_length_m > 20.0


def test_tracks_plan_and_arrives(flight):
    _path, res = flight
    assert res.final_error_m < 1.0
    assert res.max_track_error_m < 5.0


def test_real_deposition_recomputed(flight):
    _path, res = flight
    assert 0.0 < res.actual.coverage_ratio < 1.0
    # 真飞沉积与计划沉积同量级（跟踪误差带来的偏差有限）
    assert abs(res.actual.coverage_ratio - res.planned.coverage_ratio) < 0.2
    assert res.positions.shape[1] == 3
    assert len(res.times) > 100
