"""故障脚本与参考轨迹的单元测试（纯 numpy）。"""

from __future__ import annotations

import numpy as np
import pytest

from uav_neural_fault_tolerant_control import faults as FL
from uav_neural_fault_tolerant_control import trajectory as T

HOVER = 2.33


def test_healthy_plan_has_unit_gains():
    plan = FL.FaultPlan(kind="healthy")
    assert np.allclose(plan.gains(0.0), 1.0)
    assert np.allclose(plan.gains(10.0), 1.0)
    assert np.allclose(plan.bias(10.0, HOVER), 0.0)


def test_loss_only_after_onset():
    plan = FL.FaultPlan(kind="loss", rotor=1, onset=2.0)
    assert np.allclose(plan.gains(1.9), 1.0)
    assert plan.gains(2.0 + plan.ramp + 1e-6)[1] < FL.GAIN_FLOOR + 1e-6
    # 其它桨不受影响
    others = [i for i in range(4) if i != 1]
    assert np.allclose(plan.gains(3.0)[others], 1.0)


def test_ramp_is_monotonic():
    plan = FL.FaultPlan(kind="loss", rotor=0, onset=2.0, ramp=0.4)
    values = [plan.gains(2.0 + dt)[0] for dt in (0.0, 0.1, 0.2, 0.4)]
    assert all(a >= b - 1e-12 for a, b in zip(values, values[1:]))
    assert values[0] > values[-1]


def test_degradation_severity_is_final_gain_loss():
    plan = FL.FaultPlan(kind="degradation", rotor=2, onset=2.0, severity=0.5, ramp=0.0)
    assert np.isclose(plan.gains(3.0)[2], 0.5, atol=1e-6)


def test_stuck_rotor_keeps_constant_bias():
    plan = FL.FaultPlan(kind="stuck", rotor=1, onset=2.0, ramp=0.0)
    before = plan.bias(1.0, HOVER)
    after = plan.bias(3.0, HOVER)
    assert np.allclose(before, 0.0)
    assert np.isclose(after[1], HOVER)
    assert np.isclose(plan.gains(3.0)[1], 0.0)


def test_dual_loss_zeros_two_rotors():
    plan = FL.FaultPlan(kind="dual_loss", rotor=0, rotor2=2, onset=2.0, ramp=0.0)
    gains = plan.gains(3.0)
    assert gains[0] < FL.GAIN_FLOOR + 1e-6
    assert gains[2] < FL.GAIN_FLOOR + 1e-6
    assert np.isclose(gains[1], 1.0)


def test_progressive_decays_per_second():
    plan = FL.FaultPlan(kind="progressive", rotor=0, onset=2.0, severity=0.2, ramp=0.0)
    assert np.isclose(plan.gains(2.0)[0], 1.0, atol=1e-6)
    assert np.isclose(plan.gains(3.0)[0], 0.8, atol=1e-6)
    assert np.isclose(plan.gains(4.0)[0], 0.6, atol=1e-6)


@pytest.mark.parametrize("kwargs", [
    {"kind": "nope"},
    {"kind": "loss", "rotor": 4},
    {"kind": "loss", "onset": -1.0},
    {"kind": "loss", "ramp": -0.5},
    {"kind": "progressive", "severity": 0.0},
    {"kind": "dual_loss", "rotor": 1, "rotor2": 1},
])
def test_invalid_plans_raise(kwargs):
    with pytest.raises(ValueError):
        FL.FaultPlan(**kwargs)


def test_default_plans_are_valid():
    assert len(FL.DEFAULT_PLANS) >= 5
    for plan in FL.DEFAULT_PLANS:
        assert plan.label()


def test_trajectory_shapes_and_reference_batch():
    for kind in T.TRAJECTORIES:
        p, v, a, yaw = T.reference(kind, 1.0)
        assert p.shape == (3,) and v.shape == (3,) and a.shape == (3,)
        assert isinstance(yaw, float)
        batch = T.reference_batch(kind, np.array([0.0, 0.5, 1.0]))
        assert batch[0].shape == (3, 3)


def test_hover_trajectory_is_constant():
    p0, _, _, _ = T.reference("hover", 0.0)
    p1, _, _, _ = T.reference("hover", 5.0)
    assert np.allclose(p0, p1)


def test_circle_trajectory_stays_on_radius():
    """绕 (0, radius) 转圈：起点在原点，所以判据是到圆心的距离恒等于半径。"""
    radius = 0.7
    pts = [T.reference("circle", t)[0] for t in np.linspace(0.0, 6.0, 25)]
    r = [float(np.hypot(p[0], p[1] - radius)) for p in pts]
    assert max(r) - min(r) < 1e-6
    assert abs(r[0] - radius) < 1e-6


def test_climb_trajectory_is_monotonic():
    zs = [T.reference("climb", t)[0][2] for t in np.linspace(0.0, 6.0, 25)]
    assert all(b >= a - 1e-9 for a, b in zip(zs, zs[1:]))


def test_unknown_trajectory_raises():
    with pytest.raises(ValueError):
        T.reference("nope", 0.0)
