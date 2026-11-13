#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop smoke test: tiny sortie, < 10 s wall time."""
from __future__ import annotations

import json

import pytest

# Skip the whole module when MuJoCo is unavailable, so a minimal CI image
# (numpy/matplotlib/pyyaml/pytest only) can still run the algorithm layer.
# exc_type=ImportError: 装了 mujoco 但加载失败（缺 DLL / 平台不兼容）也降级为 skip，
# 而不是让整个文件 collection error。
pytest.importorskip("mujoco", exc_type=ImportError)

from uav_delivery_route.core.cases import Case, Station
from uav_delivery_route.core.energy import EnergyModel
from uav_delivery_route.core.planner import RouteConfig, plan_routes

# 修好物理时钟后墙钟约为仿真时长的 1/70，真实 20 s 悬停也能秒级跑完，
# 直接用与主程序一致的悬停时长
CFG = RouteConfig(hover_s=20.0)


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


def test_physics_clock_matches_report(report):
    # 回归：曾把 step_with_control 的 n（控制周期数）当物理子步用，
    # 物理时钟比报告时间快 4 倍。每轮只推进一个控制周期后两者必须一致
    assert report.physics_time_s > 0.0
    assert abs(report.physics_time_s - report.sim_duration_s) < 1e-6


def test_fly_two_station_sortie(report):
    # 3 段 leg（中心->S1->S2->中心），到点误差小、实际航程接近计划
    assert len(report.legs) == 3
    assert report.mean_arrival_error_m < 0.5
    assert report.max_arrival_error_m < 1.0
    ratio = report.actual_total_m / report.planned_total_m
    assert 0.9 < ratio < 1.15
    assert report.sim_duration_s > 0 and report.wall_time_s < 30.0


def test_flight_report_serializable(report):
    d = report.to_dict()
    s = json.dumps(d)
    assert "actual_total_dist_m" in s and "arrival_error_m" in s
