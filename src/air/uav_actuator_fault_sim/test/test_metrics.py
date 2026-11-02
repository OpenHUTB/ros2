# -*- coding: utf-8 -*-
"""指标计算单元测试：用手工造出的轨迹验证每个派生量。"""

import numpy as np
import pytest

from uav_actuator_fault_sim.metrics import (
    CRASH_ALTITUDE,
    FlightResult,
    aggregate,
    group_summary,
)

DT = 0.01


def _make(pos_z, vx=0.0, vz=0.0, onset=2.0, scenario="single_loss", scene="open_ground"):
    """按高度序列造一条轨迹；pos_z / vz 可以是标量也可以逐点给。"""
    n = len(pos_z)
    t = np.arange(n) * DT
    pos = np.zeros((n, 3))
    pos[:, 0] = vx * t
    pos[:, 2] = pos_z
    vel = np.zeros((n, 3))
    vel[:, 0] = vx
    vel[:, 2] = vz
    return FlightResult(
        t=t,
        pos=pos,
        vel=vel,
        omega=np.zeros((n, 3)),
        euler=np.zeros((n, 3)),
        thrusts=np.full((n, 4), 2.0),
        health=np.ones((n, 4)),
        target=np.array([0.0, 0.0, 1.5]),
        onset=onset,
        scenario=scenario,
        scene=scene,
    )


# --------------------------------------------------------------------------- #
def test_healthy_hover_is_not_a_crash():
    r = _make(np.full(200, 1.5), scenario="healthy", onset=None)
    assert not r.crashed
    assert r.crash_index is None
    assert r.crash_delay is None
    assert r.touchdown_speed is None
    assert not r.safe_landing
    assert r.position_rmse == pytest.approx(0.0, abs=1e-9)


def test_crash_detection_and_delay():
    # 前 200 步 1.5 m 悬停，之后自由下落，第 100 步跌破阈值
    z = np.concatenate([np.full(200, 1.5),
                        np.linspace(1.5, CRASH_ALTITUDE - 0.01, 100),
                        np.linspace(CRASH_ALTITUDE, -2.0, 100)])
    r = _make(z, onset=2.0)
    assert r.crashed
    assert r.crash_index is not None
    # 故障注入在 t=2.0（第 200 点），坠地应在之后
    assert r.crash_delay is not None and r.crash_delay > 0.0
    assert r.min_altitude < 0.0


def test_touchdown_speed_uses_velocity_at_crash_index():
    n = 120
    z = np.linspace(1.5, -1.0, n)
    r = _make(z, vz=-3.0, onset=0.0)
    k = r.crash_index
    assert k is not None
    assert r.touchdown_speed == pytest.approx(np.linalg.norm(r.vel[k]))
    assert r.touchdown_speed == pytest.approx(3.0, rel=1e-6)


def test_safe_landing_requires_slow_touchdown():
    slow = _make(np.linspace(1.5, 0.0, 100), vz=-0.2, onset=0.0)
    assert slow.safe_landing
    fast = _make(np.linspace(1.5, 0.0, 100), vz=-4.0, onset=0.0)
    assert not fast.safe_landing


def test_horizontal_drift_reads_last_position():
    r = _make(np.full(100, 1.5), vx=1.0, scenario="healthy", onset=None)
    # 最后一点 x = 0.99 * 1.0 = 0.99
    assert r.horizontal_drift == pytest.approx(0.99, abs=1e-6)


def test_control_effort_integrates_squared_thrust():
    n = 101
    r = _make(np.full(n, 1.5), scenario="healthy", onset=None)
    # 四桨各 2 N 常值 → Σu² = 4·4 = 16，∫ over (n-1)·dt = 1.0 s
    assert r.control_effort == pytest.approx(16.0 * (n - 1) * DT, rel=1e-6)


def test_final_yaw_rate_average_of_last_window():
    n = 100
    r = _make(np.full(n, 1.5), scenario="healthy", onset=None)
    r.omega[:, 2] = np.pi / 2.0                     # 90 deg/s
    assert r.final_yaw_rate == pytest.approx(90.0, rel=1e-6)


def test_summary_has_stable_keys():
    r = _make(np.full(100, 1.5), scenario="healthy", onset=None)
    row = r.summary()
    for key in ("scene", "scenario", "crashed", "min_altitude", "safe_landing",
                "pos_rmse", "control_effort", "final_yaw_rate"):
        assert key in row
    assert row["crashed"] == 0
    assert row["onset"] == ""          # 无故障时留空而不是 None


# --------------------------------------------------------------------------- #
def test_aggregate_counts_survival():
    good = [_make(np.full(100, 1.5), scenario="healthy", onset=None) for _ in range(3)]
    bad = _make(np.linspace(1.5, -5.0, 100), vz=-5.0, onset=0.0)
    agg = aggregate(good + [bad])
    assert agg["n"] == 4
    assert agg["survival_rate"] == pytest.approx(0.75)
    assert agg["mean_min_alt"] < 1.5
    assert agg["mean_crash_delay"] >= 0.0


def test_group_summary_splits_by_scenario():
    a = _make(np.full(100, 1.5), scenario="healthy", onset=None)
    b = _make(np.linspace(1.5, -5.0, 100), vz=-5.0, scenario="single_loss", onset=0.0)
    g = group_summary([a, b], key="scenario")
    assert set(g) == {"healthy", "single_loss"}
    assert g["healthy"]["survival_rate"] == pytest.approx(1.0)
    assert g["single_loss"]["survival_rate"] == pytest.approx(0.0)


def test_aggregate_rejects_empty():
    with pytest.raises(ValueError):
        aggregate([])
