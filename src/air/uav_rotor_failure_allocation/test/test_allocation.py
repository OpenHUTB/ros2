"""检查推力分配、偏航权限、单桨失效后的可达力矩集合.

所有期望值都能手算: X 型布局的分配矩阵、mg/4 的悬停解、κ·mg 的被动偏航、
4·f_max 与 2·f_max 的零力矩推力上限、菱形/三角形的解析面积.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_rotor_failure_allocation.core import QuadrotorAllocator  # noqa: E402

DIAMOND_EXTENT = 0.25 * (2.0 * 9.80665) / math.sqrt(2.0)  # L·mg/√2 ≈ 3.467 N·m


@pytest.fixture
def quad():
    return QuadrotorAllocator(mass=2.0, arm_length=0.25, torque_ratio=0.016)


def test_allocation_matrix_maps_thrust_to_wrench(quad):
    matrix = quad.allocation_matrix
    assert matrix.shape == (4, 4)
    wrench = matrix @ np.array([1.0, 1.0, 1.0, 1.0])
    assert wrench[0] == pytest.approx(4.0)
    assert wrench[1] == pytest.approx(0.0, abs=1e-12)
    assert wrench[2] == pytest.approx(0.0, abs=1e-12)
    assert wrench[3] == pytest.approx(0.0, abs=1e-12)


def test_hover_allocation_is_mg_over_four(quad):
    thrusts, feasible = quad.hover_allocation()
    assert feasible
    assert thrusts == pytest.approx([quad.hover_thrust / 4.0] * 4)
    assert quad.wrench(thrusts) == pytest.approx([quad.hover_thrust, 0.0, 0.0, 0.0], abs=1e-9)
    assert quad.thrust_to_weight == pytest.approx(2.0)


def test_allocation_roundtrip(quad):
    rng = np.random.default_rng(7)
    for _ in range(20):
        wrench = np.array([rng.uniform(0, 39.0), rng.uniform(-3.0, 3.0),
                           rng.uniform(-3.0, 3.0), rng.uniform(-0.3, 0.3)])
        thrusts, _ = quad.allocate(wrench)
        assert quad.wrench(thrusts) == pytest.approx(wrench, abs=1e-9)


def test_thrust_limit_is_detected(quad):
    thrusts, feasible = quad.allocate([4.0 * quad.max_thrust + 1.0, 0.0, 0.0, 0.0])
    assert not feasible
    assert np.max(thrusts) > quad.max_thrust
    thrusts, feasible = quad.allocate([-1.0, 0.0, 0.0, 0.0])
    assert not feasible


def test_zero_moment_thrust_limits(quad):
    # 完好时四桨同时满推力: 4·f_max = 2·mg (T/W = 2)
    assert quad.max_thrust_zero_moment() == pytest.approx(4.0 * quad.max_thrust)
    assert quad.max_thrust_zero_moment() == pytest.approx(2.0 * quad.hover_thrust)
    # 单桨失效后只剩一条对角线有效, 上限正好是一半
    for failed in range(4):
        assert quad.max_thrust_zero_moment(failed) == pytest.approx(2.0 * quad.max_thrust)


def test_failure_hover_solution_and_forced_yaw(quad):
    hover = quad.hover_thrust
    for failed in range(4):
        thrusts, feasible, achieved = quad.allocate_with_failure([hover, 0.0, 0.0, 0.0], failed)
        assert feasible
        assert thrusts[failed] == 0.0
        # 剩余两桨在一条对角线上各出 mg/2, 恰好等于 f_max (T/W = 2 时); 第三个桨为 0
        active = [i for i in range(4) if i != failed]
        assert sum(1 for i in active if thrusts[i] <= 1e-9) == 1
        driving = [thrusts[i] for i in active if thrusts[i] > 1e-9]
        assert driving == pytest.approx([quad.max_thrust, quad.max_thrust], rel=1e-9)
        assert achieved[0] == pytest.approx(hover)
        assert achieved[1] == pytest.approx(0.0, abs=1e-9)
        assert achieved[2] == pytest.approx(0.0, abs=1e-9)
        # 两桨同向旋转, 反扭矩叠加成 κ·mg, 无法靠剩余桨抵消
        assert abs(achieved[3]) == pytest.approx(quad.torque_ratio * hover)
        assert quad.forced_yaw_after_failure(failed) == pytest.approx(achieved[3])


def test_forced_yaw_sign_alternates(quad):
    signs = [np.sign(quad.forced_yaw_after_failure(failed)) for failed in range(4)]
    assert signs == [1.0, -1.0, 1.0, -1.0]


def test_failure_cannot_hover_with_low_thrust_to_weight():
    # T/W = 1.6: 单桨失效后零力矩最大推力只有 0.8·mg, 悬停不可行
    weak = QuadrotorAllocator(mass=2.0, arm_length=0.25, torque_ratio=0.016,
                              max_thrust=1.6 * 2.0 * 9.80665 / 4.0)
    assert weak.max_thrust_zero_moment(0) == pytest.approx(0.8 * weak.hover_thrust)
    _, feasible, _ = weak.allocate_with_failure([weak.hover_thrust, 0.0, 0.0, 0.0], 0)
    assert not feasible
    # 完好时同一架飞机仍能悬停
    assert weak.hover_allocation()[1]


def test_max_yaw_torque_formula(quad):
    hover = quad.hover_thrust
    kappa = quad.torque_ratio
    # 总推力不超过一组两桨的上限时, 全部推力压在同向的一组上: τz = κ·T
    assert quad.max_yaw_torque(0.0) == pytest.approx(0.0)
    assert quad.max_yaw_torque(hover / 2.0) == pytest.approx(kappa * hover / 2.0)
    assert quad.max_yaw_torque(hover) == pytest.approx(kappa * hover)
    # 超过一组上限后, 另一组必须出力, 权限随推力线性下降
    assert quad.max_yaw_torque(1.75 * hover) == pytest.approx(0.25 * kappa * hover)
    assert quad.max_yaw_torque(2.0 * hover) == pytest.approx(0.0, abs=1e-12)
    with pytest.raises(ValueError):
        quad.max_yaw_torque(-1.0)
    with pytest.raises(ValueError):
        quad.max_yaw_torque(2.5 * hover)


def test_healthy_feasible_set_is_diamond(quad):
    polygon = quad.feasible_polygon()
    assert polygon.shape == (4, 2)
    # 顶点在坐标轴上: (±L·mg/√2, 0) 与 (0, ±L·mg/√2)
    radius = DIAMOND_EXTENT
    assert np.allclose(np.abs(polygon).max(axis=1), radius, atol=1e-9)
    assert quad.polygon_extents(polygon)[0] == pytest.approx(radius)
    # 菱形面积 = 2·a²
    assert quad.polygon_area(polygon) == pytest.approx(2.0 * radius ** 2, rel=1e-9)
    # 中心对称
    assert polygon.mean(axis=0) == pytest.approx([0.0, 0.0], abs=1e-12)


def test_failed_feasible_set_is_triangle(quad):
    for failed in range(4):
        polygon = quad.feasible_polygon(failed_rotor=failed)
        assert polygon.shape == (3, 2)
        assert quad.polygon_area(polygon) == pytest.approx(
            0.5 * DIAMOND_EXTENT ** 2, rel=1e-9)


def test_feasible_set_matches_allocation(quad):
    rng = np.random.default_rng(11)
    for failed in (None, 2):
        polygon = quad.feasible_polygon(failed_rotor=failed)
        radius = 1.1 * max(quad.polygon_extents(polygon))
        for _ in range(2000):
            point = rng.uniform(-radius, radius, 2)
            request = [quad.hover_thrust, point[0], point[1], 0.0]
            if failed is None:
                _, feasible = quad.allocate(request)
            else:
                _, feasible, _ = quad.allocate_with_failure(request, failed)
            assert bool(feasible) == quad.polygon_contains(polygon, point), point


def test_feasible_set_shrinks_towards_the_yaw_limit(quad):
    hover = quad.hover_thrust
    limit = quad.torque_ratio * hover
    wide = quad.feasible_polygon(total_thrust=hover, yaw=0.0)
    narrow = quad.feasible_polygon(total_thrust=hover, yaw=0.9 * limit)
    assert quad.polygon_area(narrow) < quad.polygon_area(wide)
    # 达到偏航上限时只剩一个解: 同向两桨满推力
    edge = quad.feasible_polygon(total_thrust=hover, yaw=limit)
    assert edge.shape[0] == 1
    assert edge[0] == pytest.approx([0.0, 0.0], abs=1e-9)


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError):
        QuadrotorAllocator(mass=0.0)
    with pytest.raises(ValueError):
        QuadrotorAllocator(arm_length=-0.1)
    with pytest.raises(ValueError):
        QuadrotorAllocator(torque_ratio=0.0)
    with pytest.raises(ValueError):
        QuadrotorAllocator(max_thrust=-1.0)
    with pytest.raises(ValueError):
        QuadrotorAllocator().allocate([1.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        QuadrotorAllocator().max_thrust_zero_moment(4)
