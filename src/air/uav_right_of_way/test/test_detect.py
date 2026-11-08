#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侵入判定（core.detect）单元测试：解析 CPA 与威胁触发条件。"""
import numpy as np
import pytest

from uav_right_of_way.core.detect import (AircraftState, DetectParams, Threat,
                                          cpa, fleet_threats, predict_threat,
                                          threats_against)

DP = DetectParams(horizon_s=8.0, cpa_threshold_m=40.0)


def state(uid, pos, vel, priority=0):
    return AircraftState(uid, np.asarray(pos, float),
                         np.asarray(vel, float), priority)


def test_cpa_headon_analytic():
    # 100 m 外以 10 m/s 迎面接近 → t_cpa=10 s，最近距离 0
    t, d = cpa(np.array([100.0, 0, 0]), np.array([-10.0, 0, 0]), 30.0)
    assert t == pytest.approx(10.0)
    assert d == pytest.approx(0.0, abs=1e-9)


def test_cpa_relative_stationary():
    t, d = cpa(np.array([30.0, 40.0, 0]), np.zeros(3), 8.0)
    assert t == pytest.approx(0.0)
    assert d == pytest.approx(50.0)


def test_cpa_offset_crossing():
    # 横向偏 50 m 的交叉：最近距离恰为 50 m，出现在 t=10 s
    t, d = cpa(np.array([100.0, 50.0, 0]), np.array([-10.0, 0, 0]), 30.0)
    assert t == pytest.approx(10.0)
    assert d == pytest.approx(50.0)


def test_cpa_clamped_to_horizon():
    t, d = cpa(np.array([100.0, 0, 0]), np.array([-10.0, 0, 0]), 5.0)
    assert t == pytest.approx(5.0)
    assert d == pytest.approx(50.0)


def test_threat_triggers_within_horizon():
    # 150 m 外 20 m/s 迎面：t_cpa=7.5 s ≤ 8 s，CPA=0 → 触发
    own = state("A", (0, 0, 15), (10, 0, 0))
    other = state("B", (150, 0, 15), (-10, 0, 0))
    th = predict_threat(own, other, DP)
    assert th is not None
    assert th.other == "B"
    assert th.cpa_t == pytest.approx(7.5)
    assert th.cpa_dist < 1e-6


def test_no_threat_beyond_horizon():
    # 200 m 外 20 m/s：t_cpa=10 s > 8 s → 不触发
    own = state("A", (0, 0, 15), (10, 0, 0))
    other = state("B", (200, 0, 15), (-10, 0, 0))
    assert predict_threat(own, other, DP) is None


def test_no_threat_when_cpa_far():
    # 平行航迹横向 100 m：CPA 距离 100 m > 40 m → 不触发
    own = state("A", (0, 0, 15), (10, 0, 0))
    other = state("B", (50, 100, 15), (10, 0, 0))
    assert predict_threat(own, other, DP) is None


def test_no_threat_when_separating():
    # 已经越过最近点、正在远离：即使距离 < 40 m 也不是威胁
    own = state("A", (0, 0, 15), (10, 0, 0))
    other = state("B", (30, 0, 15), (10, 0, 0))  # B 在前方同向同速→静止相对
    # 相对静止且距离 30 < 40 → 触发（静止不算"远离"）
    assert predict_threat(own, other, DP) is not None
    # B 在前方且更快 → 拉开距离 → 不触发
    other_fast = state("B", (30, 0, 15), (12, 0, 0))
    assert predict_threat(own, other_fast, DP) is None


def test_threat_already_inside_threshold():
    own = state("A", (0, 0, 15), (0, 0, 0))
    other = state("B", (30, 0, 15), (0, 0, 0))
    th = predict_threat(own, other, DP)
    assert th is not None
    assert th.cpa_t == pytest.approx(0.0)
    assert th.cpa_dist == pytest.approx(30.0)


def test_fleet_threats_pairwise():
    a = state("A", (0, 0, 15), (10, 0, 0))
    b = state("B", (100, 0, 15), (-10, 0, 0))
    c = state("C", (0, 500, 15), (0, 10, 0))  # 远离 others
    out = fleet_threats([c, b, a], DP)
    assert set(out) == {("A", "B")}
    near = threats_against(a, [b, c], DP)
    assert list(near) == ["B"]
    assert isinstance(near["B"], Threat)


def test_detect_params_validation():
    with pytest.raises(ValueError):
        DetectParams(horizon_s=0.0)
    with pytest.raises(ValueError):
        DetectParams(cpa_threshold_m=-1.0)
