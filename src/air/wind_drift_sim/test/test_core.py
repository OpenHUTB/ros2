#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wind_drift_sim 单元测试（不依赖 ROS）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import (  # noqa: E402
    WindDriftPlant, drift_metrics, no_compensation, pd_compensation, simulate,
)


def test_no_compensation_drifts():
    """3m/s 定常风、无补偿时无人机应被吹偏。"""
    ts, xs = simulate(no_compensation, wind=(3.0, 0.0), duration=20.0)
    m = drift_metrics(xs)
    assert m['final_drift'] > 1.0


def test_no_compensation_reaches_terminal_velocity():
    """稳态时相对气流速度应为 0, 即随风速漂移: v ≈ wind。"""
    ts, xs = simulate(no_compensation, wind=(3.0, 0.0), duration=60.0)
    # 最后 1s 位移 ≈ 3m/s * 1s = 3m
    (x1, _), (x2, _) = xs[-100], xs[-1]
    speed = (x2 - x1) / (0.02 * 99)
    assert abs(speed - 3.0) < 0.1


def test_pd_compensation_reduces_drift():
    """PD 位置补偿应显著减小漂移 (纯 PD 对常值风有少量稳态偏差,
    完全消除需积分项, 见 docs 说明)。"""
    _, xs_none = simulate(no_compensation, duration=30.0)
    _, xs_pd = simulate(pd_compensation(kp=0.8, kd=1.2), duration=30.0)
    m_none = drift_metrics(xs_none)
    m_pd = drift_metrics(xs_pd)
    assert m_pd['max_drift'] < m_none['max_drift'] * 0.5
    assert m_pd['final_drift'] < 2.0  # 稳态偏差 ~1/kp = 1.25m, 远小于 90m 自由漂移


def test_stronger_wind_more_drift():
    _, xs_light = simulate(no_compensation, wind=(2.0, 0.0), duration=30.0)
    _, xs_strong = simulate(no_compensation, wind=(6.0, 0.0), duration=30.0)
    assert drift_metrics(xs_strong)['final_drift'] > \
        drift_metrics(xs_light)['final_drift']


def test_plant_deterministic():
    p1 = WindDriftPlant()
    p2 = WindDriftPlant()
    for _ in range(100):
        p1.step((0.0, 0.0), 0.02)
        p2.step((0.0, 0.0), 0.02)
    assert p1.pos == p2.pos
