"""分配器与车辆模型的单元测试。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from torque_vectoring_yaw_control.allocator import allocate, allocate_equal
from torque_vectoring_yaw_control.vehicle import VehicleSpec, WheelState


ARMS = WheelState.moment_arms(1.56)


def test_equal_caps_split_evenly():
    r = allocate(4000.0, 0.0, [3000.0] * 4, ARMS)
    assert r.fx == pytest.approx([1000.0] * 4, abs=1e-6)
    assert r.achieved_total == pytest.approx(4000.0, rel=1e-9)
    assert not r.saturated


def test_moment_request_is_met_exactly():
    r = allocate(4000.0, 2000.0, [3000.0] * 4, ARMS)
    assert r.achieved_total == pytest.approx(4000.0, rel=1e-9)
    assert r.achieved_moment == pytest.approx(2000.0, rel=1e-9)
    # 正力矩 = 逆时针 = 右侧推力更大
    assert r.fx[1] > r.fx[0] and r.fx[3] > r.fx[2]


def test_symmetric_moment_forces_left_right_balance():
    """横摆力矩为零时，左右总力必须相等——这是几何约束，不是取舍。"""
    r = allocate(4000.0, 0.0, [1500.0, 3500.0, 1500.0, 3500.0], ARMS)
    assert r.fx[0] + r.fx[2] == pytest.approx(r.fx[1] + r.fx[3], rel=1e-9)


def test_high_capacity_wheel_takes_more():
    r = allocate(4000.0, 2000.0, [3000.0, 3000.0, 300.0, 300.0], ARMS)
    assert r.fx[1] > r.fx[0]
    assert abs(r.fx[2]) <= 300.0 + 1e-6


def test_saturation_respects_caps():
    r = allocate(9000.0, 1000.0, [1500.0, 2500.0, 1500.0, 2500.0], ARMS)
    for f, c in zip(r.fx, [1500.0, 2500.0, 1500.0, 2500.0]):
        assert abs(f) <= c + 1e-6
    assert r.saturated


def test_zero_cap_wheel_is_skipped():
    r = allocate(3000.0, 0.0, [3000.0, 3000.0, 3000.0, 0.0], ARMS)
    assert r.fx[3] == 0.0
    assert r.achieved_total == pytest.approx(3000.0, rel=1e-6)


def test_all_dead_returns_zero():
    r = allocate(3000.0, 0.0, [0.0] * 4, ARMS)
    assert r.fx == [0.0] * 4
    assert r.saturated


def test_allocation_is_optimal_under_coupling():
    """随机可行解打不过解析解——验证加权最小二乘确实在约束下最优。"""
    caps = [2600.0, 2600.0, 2600.0, 2600.0]
    demand, moment = 3200.0, 900.0
    r = allocate(demand, moment, caps, ARMS)
    cost = sum((f / c) ** 2 for f, c in zip(r.fx, caps))

    # 可行域的参数化：左右总力由两个约束唯一确定，只剩"左右各自前后怎么分"
    # 两个自由度，于是可以直接撒点比较成本
    a = ARMS[1]
    left = (demand - moment / a) / 2.0
    right = (demand + moment / a) / 2.0
    rng = np.random.default_rng(1)
    best = None
    for _ in range(20000):
        u = rng.uniform(0.0, 1.0)   # 左前占比
        v = rng.uniform(0.0, 1.0)   # 右前占比
        f = [left * u, right * v, left * (1 - u), right * (1 - v)]
        if max(abs(x) for x in f) > 2600.0:
            continue
        c = float(np.sum((np.asarray(f) / np.asarray(caps)) ** 2))
        best = c if best is None else min(best, c)
    assert best is not None
    assert cost <= best + 1e-9


def test_equal_allocator_saturates_low_grip_side():
    r = allocate_equal(-4000.0, [1900.0, 2700.0, 1900.0, 2700.0])
    assert r.fx == pytest.approx([-1000.0] * 4)
    r2 = allocate_equal(-8000.0, [1900.0, 2700.0, 1900.0, 2700.0])
    assert r2.fx[0] == pytest.approx(-1900.0)   # 低附着侧先饱和
    assert r2.fx[1] == pytest.approx(-2000.0)   # 高附着侧还有余量，不了用满


def test_vehicle_reference_is_finite_and_signed():
    spec = VehicleSpec()
    assert spec.yaw_rate_reference(20.0, 0.1) > 0
    assert spec.yaw_rate_reference(20.0, -0.1) < 0
    assert math.isfinite(spec.yaw_rate_reference(0.0, 0.1))


def test_yaw_rate_limit_follows_grip():
    spec = VehicleSpec()
    assert spec.yaw_rate_limit(10.0) == pytest.approx(spec.mu * 9.80665 / 10.0)
    assert spec.yaw_rate_limit(30.0) < spec.yaw_rate_limit(10.0)


def test_force_cap_shrinks_with_lateral_force():
    spec = VehicleSpec()
    load = 4000.0
    assert spec.force_cap(load, 0.0) == pytest.approx(spec.mu * load)
    assert spec.force_cap(load, 0.8 * spec.mu * load) < spec.force_cap(load, 0.0)
    assert spec.force_cap(load, spec.mu * load) == pytest.approx(0.0, abs=1e-6)


def test_load_transfer_moves_weight_forward_on_braking():
    spec = VehicleSpec()
    f0, r0 = spec.axle_loads(0.0)
    f1, r1 = spec.axle_loads(-5.0)
    assert f1 > f0 and r1 < r0
    assert f1 + r1 == pytest.approx(f0 + r0, rel=1e-6)


def test_understeer_gradient_sign_matches_parameters():
    spec = VehicleSpec()
    # 前轴侧偏刚度低于后轴 → 不足转向（K > 0）
    stiff_rear = VehicleSpec(cf=70000.0, cr=140000.0)
    assert stiff_rear.understeer_gradient() > 0
    loose_rear = VehicleSpec(cf=140000.0, cr=70000.0)
    assert loose_rear.understeer_gradient() < 0
    assert math.isfinite(spec.understeer_gradient())
