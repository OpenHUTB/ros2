#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让路恢复（core.recover）单元测试：余弦剖面性质与恢复期监视。"""
import math

import numpy as np
import pytest

from uav_right_of_way.core.detect import AircraftState, DetectParams
from uav_right_of_way.core.recover import RecoveryProfile, should_abort

DP = DetectParams(horizon_s=8.0, cpa_threshold_m=40.0)


def state(uid, pos, vel):
    return AircraftState(uid, np.asarray(pos, float),
                         np.asarray(vel, float), 0)


def test_profile_endpoints():
    rec = RecoveryProfile(lateral0=20.0, alt0=-10.0, speed_factor0=0.7,
                          duration=10.0)
    assert rec.lateral(0.0) == pytest.approx(20.0)
    assert rec.lateral(1.0) == pytest.approx(0.0, abs=1e-12)
    assert rec.alt(0.0) == pytest.approx(-10.0)
    assert rec.alt(1.0) == pytest.approx(0.0, abs=1e-12)
    assert rec.speed_factor(0.0) == pytest.approx(0.7)
    assert rec.speed_factor(1.0) == pytest.approx(1.0)


def test_profile_zero_rate_at_ends():
    rec = RecoveryProfile(lateral0=20.0, alt0=0.0, duration=10.0)
    assert rec.lateral_rate(0.0) == pytest.approx(0.0, abs=1e-12)
    assert rec.lateral_rate(1.0) == pytest.approx(0.0, abs=1e-12)
    # 中点变化率最大
    assert abs(rec.lateral_rate(0.5)) > abs(rec.lateral_rate(0.25))


def test_profile_midpoint_half_and_monotonic():
    rec = RecoveryProfile(lateral0=20.0, alt0=0.0, duration=10.0)
    assert rec.lateral(0.5) == pytest.approx(10.0)
    vals = [rec.lateral(s / 20.0) for s in range(21)]
    assert all(a >= b - 1e-12 for a, b in zip(vals, vals[1:]))


def test_profile_integrates_to_zero_offset():
    # 对横向变化率做中点法数值积分，终点偏移应为 0（余弦剖面自洽）
    rec = RecoveryProfile(lateral0=25.0, alt0=0.0, duration=12.0)
    n = 2000
    dt = rec.duration / n
    x = rec.lateral0
    for k in range(n):
        x += rec.lateral_rate((k + 0.5) / n) * dt
    assert x == pytest.approx(0.0, abs=1e-4)


def test_profile_validation():
    with pytest.raises(ValueError):
        RecoveryProfile(lateral0=1.0, alt0=0.0, duration=0.0)


def test_should_abort_detects_new_threat():
    own = state("A", (0, 0, 15), (8, 0, 0))
    calm = state("B", (0, 200, 15), (8, 0, 0))       # 平行 200 m，无威胁
    assert should_abort(own, [calm], DP) is None
    intruder = state("C", (100, 0, 15), (-8, 0, 0))  # 迎面 → 恢复应中止
    th = should_abort(own, [calm, intruder], DP)
    assert th is not None
    assert th.other == "C"
