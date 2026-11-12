"""分配器的单元测试：纯 numpy，不需要 MuJoCo / torch。

用的是实测出来的控制效率矩阵（见 docs 里的推导），以及一个轻量假 `params`
对象——它只有分配器真正读的 5 个字段。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from uav_neural_fault_tolerant_control import allocation as A
from uav_neural_fault_tolerant_control import features as F


@dataclass
class FakeParams:
    mass: float = 0.95
    arm: float = 0.106
    k_m: float = 0.015
    max_thrust: float = 6.0
    hover_thrust: float = 2.33


PARAMS = FakeParams()

#: 实测的控制效率矩阵：u（四桨推力）→ [T, τx, τy, τz]
B = np.array([
    [1.0, 1.0, 1.0, 1.0],
    [-0.106, 0.106, 0.106, -0.106],
    [-0.106, 0.106, -0.106, 0.106],
    [-0.015, -0.015, 0.015, 0.015],
])


def test_effective_thrust_roundtrip():
    u = np.array([2.0, 2.5, 3.0, 1.5])
    gains = np.array([1.0, 0.6, 1.0, 0.8])
    v = A.effective_thrust(u, gains)
    back = A.command_from_effective(v, PARAMS, gains)
    assert np.allclose(back, u)


def test_command_from_effective_handles_dead_rotor():
    """g≈0 的桨指令必须是 0，不能是 nan/inf。"""
    gains = np.array([0.0, 1.0, 1.0, 1.0])
    v = np.array([2.33, 2.33, 2.33, 2.33])
    u = A.command_from_effective(v, PARAMS, gains)
    assert u[0] == 0.0
    assert np.all(np.isfinite(u))


def test_clamp_thrusts_bounds():
    u = A.clamp_thrusts(np.array([-1.0, 0.0, 6.5, 100.0]), PARAMS)
    assert u.tolist() == [0.0, 0.0, 6.0, 6.0]


def test_naive_redistribute_keeps_total_thrust():
    """朴素补偿：把失效桨分不到的推力均摊给其余桨，有效总推力不变。"""
    gains = np.array([0.0, 1.0, 1.0, 1.0])
    honest = np.array([2.33, 2.33, 2.33, 2.33])
    u = A.naive_redistribute(honest, PARAMS, gains)
    # 坏桨到底下发多少指令不重要（有效推力恒为 0），关键是总推力守恒
    assert np.isclose(A.effective_thrust(u, gains).sum(), honest.sum())


def test_ls_matches_desired_when_healthy():
    """四桨健康、期望量可行时，最小二乘应当基本满足期望（有 λ 正则，故允许 1e-3）。"""
    desired = A.effective_thrust(np.array([2.33, 2.33, 2.33, 2.33]), np.ones(4))
    gains = np.ones(4)
    u = A.ls_allocation(B, B @ desired, PARAMS, gains)
    assert np.allclose(u, desired, atol=1e-3)
    assert np.allclose(B @ A.effective_thrust(u, gains), B @ desired, atol=1e-3)


def test_box_bound_uses_gain_times_max_thrust():
    """回归测试：有效推力的上界是 g_i·T_max，不是 T_max。

    曾经把上界写成 T_max，结果在退化桨上允许了不可能的有效推力，
    QP 反而比最小二乘更差（自检发现）。
    """
    gains = np.array([0.7, 0.5, 0.0, 0.7])
    desired = np.array([7.0, 0.5, 0.5, 0.0])
    u = A.solve_box_qp(B, desired, PARAMS, gains)
    v = A.effective_thrust(u, gains)
    assert np.all(v >= -1e-9)
    assert np.all(v <= gains * PARAMS.max_thrust + 1e-6)


def test_qp_not_worse_than_ls():
    """精确枚举解的目标函数不劣于最小二乘解（自检里也查同一条）。"""
    rng = np.random.default_rng(0)
    w = A._weights(None)
    for _ in range(40):
        gains = rng.uniform(0.2, 1.0, size=4)
        if rng.random() < 0.4:
            gains[rng.integers(0, 4)] = 0.0
        desired = np.array([rng.uniform(3.0, 8.0),
                            rng.uniform(-0.6, 0.6),
                            rng.uniform(-0.6, 0.6),
                            rng.uniform(-0.05, 0.05)])
        u_ls = A.ls_allocation(B, desired, PARAMS, gains)
        u_qp = A.solve_box_qp(B, desired, PARAMS, gains)

        def cost(u):
            v = A.effective_thrust(u, gains)
            res = B @ v - desired
            return float(res @ w @ res + A.REF_WEIGHT * np.sum(v ** 2))

        assert cost(u_qp) <= cost(u_ls) + 1e-6


def test_achievable_thrust_scales_with_health():
    assert np.isclose(A.achievable_thrust(PARAMS, np.ones(4)), 4 * 6.0)
    gains = np.array([0.0, 1.0, 1.0, 1.0])
    assert np.isclose(A.achievable_thrust(PARAMS, gains), 3 * 6.0)


def test_features_are_finite_and_normalized():
    u_ls = np.array([2.33, 2.33, 2.33, 2.33])
    gains = np.array([0.0, 1.0, 1.0, 1.0])
    desired = B @ A.effective_thrust(u_ls, gains)
    x = F.build_features(desired, gains, u_ls, PARAMS)
    assert x.shape == (F.FEATURE_DIM,)
    assert np.all(np.isfinite(x))
    assert np.all(np.abs(x) <= 1.0 + 1e-9)


def test_target_and_residual_roundtrip():
    u_ls = np.array([2.0, 2.0, 2.0, 2.0])
    u_star = np.array([2.4, 1.9, 2.1, 1.6])
    target = F.build_target(u_star, u_ls, PARAMS)
    recovered = F.apply_residual(u_ls, target, PARAMS)
    assert np.allclose(recovered, u_star, atol=1e-9)


def test_invalid_mode_raises():
    from uav_neural_fault_tolerant_control.controller import FaultTolerantController

    with pytest.raises(ValueError):
        FaultTolerantController(PARAMS, None, None, B, mode="whatever")
