#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""声学模型 core/acoustic.py：经验公式、标定、传播与能量叠加。"""
from __future__ import annotations

import math

import pytest

from uav_noise_footprint.core.acoustic import (AcousticModel, DEFAULT,
                                               energy_sum)


def test_lw_power_law_formula():
    """Lw = 10·log10(P) + K 必须逐字成立。"""
    m = AcousticModel(rotor_power=300.0, k_const=70.73)
    assert m.sound_power_level() == pytest.approx(
        10.0 * math.log10(300.0) + 70.73)
    # 功率翻 10 倍 → Lw 增 10 dB（幂律的本质）
    m10 = AcousticModel(rotor_power=3000.0, k_const=70.73)
    assert m10.sound_power_level() - m.sound_power_level() == pytest.approx(10.0)


def test_calibration_at_100m_is_55db():
    """K 的标定目标：基准机型 100 m 处约 55 dB。"""
    assert DEFAULT.calibration_check(100.0) == pytest.approx(55.0, abs=0.5)


def test_lp_matches_propagation_formula():
    """Lp(r) = Lw − 20·log10(r) − α·r 逐项成立。"""
    m = AcousticModel(rotor_power=300.0, k_const=70.73)
    r = 37.0
    expect = m.sound_power_level() - 20.0 * math.log10(r) - 0.005 * r
    assert m.lp_at(r) == pytest.approx(expect, rel=1e-9)


def test_lp_monotonic_decay():
    levels = [DEFAULT.lp_at(r) for r in (10.0, 30.0, 100.0, 300.0, 1000.0)]
    assert levels == sorted(levels, reverse=True)


def test_atmospheric_absorption_contributes():
    """300 m 处实测值应比纯球面扩散低 α·300 ≈ 1.5 dB。"""
    m = AcousticModel(rotor_power=300.0, k_const=70.73)
    pure = m.sound_power_level() - 20.0 * math.log10(300.0)
    assert (pure - m.lp_at(300.0)) == pytest.approx(1.5, abs=0.05)


def test_lp_from_3d_points():
    a, b = (0.0, 0.0, 30.0), (40.0, 0.0, 0.0)
    assert DEFAULT.lp_from(a, b) == pytest.approx(DEFAULT.lp_at(50.0))


def test_energy_sum():
    assert energy_sum([50.0, 50.0]) == pytest.approx(53.0103, abs=1e-3)
    assert energy_sum([60.0]) == pytest.approx(60.0)
    assert energy_sum([]) == -math.inf


def test_invalid_power_rejected():
    with pytest.raises(ValueError):
        DEFAULT.sound_power_level(-1.0)
