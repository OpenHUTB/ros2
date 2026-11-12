"""气动、旋翼与走廊的解析性质测试。

这些都是"闭式解 vs 数值解"或"单调性"级别的检查：不依赖仿真，
跑得快，也能在改动物理模型时立刻发现退化。
"""

from __future__ import annotations

import math

import pytest

from uav_tiltrotor_transition_corridor import (
    Corridor, RotorSpec, VehicleSpec, WingSpec, isa_density,
)
from uav_tiltrotor_transition_corridor.aero import G


# ---------------------------------------------------------------- 大气
def test_sea_level_density():
    assert isa_density(0.0) == pytest.approx(1.225, abs=2e-3)


def test_density_decreases_with_altitude():
    values = [isa_density(alt) for alt in (0.0, 1000.0, 2000.0, 4000.0, 8000.0)]
    assert all(a > b for a, b in zip(values, values[1:]))
    # 4000 m 标准值 0.8191 kg/m³
    assert isa_density(4000.0) == pytest.approx(0.8191, abs=2e-3)


def test_density_out_of_range():
    with pytest.raises(ValueError):
        isa_density(12000.0)


# ---------------------------------------------------------------- 机翼
def test_cl_linear_before_stall_and_decays_after():
    w = WingSpec()
    a_s = w.alpha_stall
    assert w.cl(a_s * 0.5) == pytest.approx(w.cl0 + w.cl_alpha * a_s * 0.5)
    assert w.cl(a_s) == pytest.approx(w.cl_max)
    assert w.cl(a_s * 1.3) < w.cl_max          # 失速后必须下降
    assert w.cl(a_s * 2.0) < w.cl(a_s * 1.3)


def test_stall_speed_matches_definition():
    spec = VehicleSpec()
    v_s = spec.stall_speed(0.0)
    lift_at_stall = 0.5 * 1.225 * v_s ** 2 * spec.wing.area * spec.wing.cl_max
    assert lift_at_stall == pytest.approx(spec.weight, rel=1e-6)


def test_drag_polar_minimum_at_expected_cl():
    w = WingSpec()
    k = 1.0 / (math.pi * w.oswald * w.aspect_ratio)
    cl_md = math.sqrt(w.cd0 / k)
    assert w.cd(cl_md) == pytest.approx(2 * w.cd0, rel=1e-9)
    assert w.cd(cl_md * 1.5) > w.cd(cl_md)


# ---------------------------------------------------------------- 旋翼
def test_hover_thrust_matches_closed_form():
    r = RotorSpec()
    rho = 1.225
    t_hover = r.hover_thrust_max(rho)
    # 由 T * v_i = P * fm，v_i = sqrt(T/(2 rho A)) 反解
    vi = math.sqrt(t_hover / (2 * rho * r.disk_area))
    assert t_hover * vi / r.figure_of_merit == pytest.approx(r.power_max, rel=1e-9)


def test_power_limited_thrust_matches_bisection():
    r, rho = RotorSpec(), 1.225
    for v in (0.0, 5.0, 10.0, 15.0):
        t = r.power_limited_thrust(v, rho)
        # 二分在推力上的容差是 1e-4 N，折算到功率约 2e-3 W
        assert r.shaft_power(t, v, rho) == pytest.approx(r.power_max, abs=0.5)


def test_max_thrust_decreases_with_speed():
    r, rho = RotorSpec(), 1.225
    ts = [r.max_thrust(v, rho) for v in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0)]
    assert all(a > b for a, b in zip(ts, ts[1:]))


def test_thrust_vanishes_at_zero_thrust_speed():
    r, rho = RotorSpec(), 1.225
    assert r.max_thrust(r.v_zero_thrust, rho) == pytest.approx(0.0, abs=1e-6)
    assert r.max_thrust(r.v_zero_thrust * 2, rho) == 0.0


def test_thrust_to_weight_is_marginal():
    """推重比 1.25 是本模块的前提：太高走廊退化，太低连悬停都保不住。"""
    spec = VehicleSpec()
    assert 1.1 < spec.thrust_to_weight < 1.4


# ---------------------------------------------------------------- 走廊
def test_hover_requires_steep_tilt():
    c = Corridor()
    assert c.feasible(0.0, 90.0)
    assert not c.feasible(0.0, 30.0)
    lo, hi = c.tilt_bounds(0.0)
    assert hi == pytest.approx(90.0)
    # 悬停时 T = W/sinδ ≤ T_max  →  sinδ ≥ W/T_max
    spec = VehicleSpec()
    expect = math.degrees(math.asin(spec.weight / spec.rotor.hover_thrust_max(1.225)))
    assert lo == pytest.approx(expect, abs=1.0)


def test_load_limit_matches_tilt_bounds_lower_edge():
    c = Corridor()
    for v in (0.0, 4.0, 8.0, 11.0):
        lo, _ = c.tilt_bounds(v)
        assert c.load_tilt_limit(v) == pytest.approx(lo, abs=1.0)


def test_corridor_pinches_before_reopening():
    """V=0 到失速速度之间，下边界先上移再回落 —— 推力衰减快于升力增长。"""
    c = Corridor()
    lo0 = c.tilt_bounds(0.0)[0]
    lo4 = c.tilt_bounds(4.0)[0]
    lo10 = c.tilt_bounds(10.0)[0]
    assert lo4 > lo0
    assert lo10 < lo4


def test_upper_bound_shrinks_at_high_speed():
    c = Corridor()
    assert c.tilt_bounds(20.0)[1] < c.tilt_bounds(12.0)[1]
    assert c.max_level_speed() == pytest.approx(23.75, abs=0.3)
    assert c.tilt_bounds(c.max_level_speed() + 3.0) is None


def test_dead_band_exists():
    """悬停极限速度低于失速速度，才需要"走廊"这个概念。"""
    c = Corridor()
    assert c.has_dead_band
    assert c.hover_speed_limit < c.stall_speed


def test_altitude_raises_the_lower_bound():
    sea = Corridor(alt_m=0.0)
    high = Corridor(alt_m=4000.0)
    assert high.stall_speed > sea.stall_speed
    assert high.hover_speed_limit < sea.hover_speed_limit
    for v in (0.0, 8.0, 10.0):
        assert high.load_tilt_limit(v) > sea.load_tilt_limit(v)


def test_lift_margin_sign():
    c = Corridor()
    assert c.lift_margin_deg(0.0, 90.0) > 0
    assert c.lift_margin_deg(0.0, 20.0) < 0
    # 机翼能独担全重时裕度不适用，用 90 表示
    assert c.lift_margin_deg(18.0, 0.0) == pytest.approx(90.0)


def test_clamp_only_pushes_up():
    c = Corridor()
    lo = c.load_tilt_limit(0.0)
    assert c.clamp_tilt(0.0, 10.0) == pytest.approx(lo + 3.0)
    # 已经在边界之上：原样返回，不被"压回来"
    assert c.clamp_tilt(0.0, 95.0) == pytest.approx(95.0)


def test_min_power_tilt_prefers_vertical_at_hover():
    c = Corridor()
    assert c.min_power_tilt(0.0) == pytest.approx(90.0, abs=2.0)


def test_level_flight_balances_forces():
    c = Corridor()
    for v, tilt in ((0.0, 90.0), (8.0, 70.0), (14.0, 40.0), (20.0, 10.0)):
        sol = c.level_flight(v, tilt)
        assert sol is not None, (v, tilt)
        lift = 0.5 * c.rho * v * v * c.spec.wing.area * sol.cl
        assert sol.thrust * math.sin(math.radians(tilt)) + lift == pytest.approx(
            c.spec.weight, rel=2e-3)
        assert sol.thrust * math.cos(math.radians(tilt)) >= sol.drag - 1e-6


def test_thrust_never_exceeds_available():
    c = Corridor()
    for v in (2.0, 6.0, 12.0, 18.0, 22.0):
        b = c.tilt_bounds(v)
        if b is None:
            continue
        sol = c.level_flight(v, b[1])
        assert sol.thrust <= c.thrust_max(v) + 1e-6
