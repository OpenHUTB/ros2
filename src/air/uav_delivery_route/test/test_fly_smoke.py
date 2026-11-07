#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop smoke test: tiny sortie, < 10 s wall time."""
from __future__ import annotations

import json

import pytest

# Skip the whole module when MuJoCo is unavailable, so a minimal CI image
# (numpy/matplotlib/pyyaml/pytest only) can still run the algorithm layer.
pytest.importorskip("mujoco")

from uav_delivery_route.core.cases import Case, Station
from uav_delivery_route.core.energy import EnergyModel
from uav_delivery_route.core.planner import RouteConfig, plan_routes

CFG = RouteConfig(hover_s=2.0)  # 压缩投递悬停，把冒烟测试控制在 10 s 内


@pytest.fixture(scope="module")
def report():
    from uav_delivery_route.core.flyer import fly_sortie
    case = Case("smoke", "2 stations",
                stations=(Station("S1", 150.0, 60.0, 0.8),
                          Station("S2", -120.0, 90.0, 0.6)))
    plan = plan_routes(case, CFG, EnergyModel())
    return fly_sortie(plan, 0, CFG)


def test_backend_imports():
    from uav_delivery_route.core.flyer import _load_backend
    Quadrotor, GeometricController, model_dir = _load_backend()
    assert Quadrotor is not None and GeometricController is not None
    assert model_dir


def test_fly_two_station_sortie(report):
    # 3 段 leg（中心->S1->S2->中心），到点误差小、实际航程接近计划
    assert len(report.legs) == 3
    assert report.mean_arrival_error_m < 2.5
    assert report.max_arrival_error_m < 4.0
    ratio = report.actual_total_m / report.planned_total_m
    assert 0.8 < ratio < 1.2
    assert report.sim_duration_s > 0 and report.wall_time_s < 30.0


def test_flight_report_serializable(report):
    d = report.to_dict()
    s = json.dumps(d)
    assert "actual_total_dist_m" in s and "arrival_error_m" in s
