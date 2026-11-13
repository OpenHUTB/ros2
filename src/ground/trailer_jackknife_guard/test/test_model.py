"""模型与场景测试。"""

from __future__ import annotations

import math

import pytest

from trailer_jackknife_guard.model import (
    CAUTION_ANGLE,
    SAFE_ANGLE,
    TrailerParams,
    guard_state,
    hitch_rate,
    max_safe_reverse_speed,
    risk_level,
    time_to_jackknife,
)
from trailer_jackknife_guard.simulation import default_scenarios, run_scenario


def test_hitch_rate_zero_when_aligned_and_straight():
    p = TrailerParams()
    assert hitch_rate(1.0, 0.0, 0.0, p) == pytest.approx(0.0)


def test_forward_motion_is_stable():
    """前进 + 右转：铰接角收敛到负平衡点，速率符号与铰接角相反。"""
    p = TrailerParams()
    delta = math.radians(6.0)
    phi_eq = math.asin(-math.tan(delta) * p.trailer_wheelbase / p.tractor_wheelbase)
    assert hitch_rate(1.0, delta, phi_eq, p) == pytest.approx(0.0, abs=1e-9)
    assert hitch_rate(1.0, delta, 0.0, p) < 0.0        # 从 0 开始变负（稳定）


def test_reverse_motion_is_unstable():
    """倒车时铰接角从 0 开始发散（速率同号），这是折叠的根源。"""
    p = TrailerParams()
    delta = math.radians(6.0)
    assert hitch_rate(-1.0, delta, 0.0, p) > 0.0
    assert hitch_rate(-1.0, delta, math.radians(30), p) > 0.0


def test_risk_levels():
    p = TrailerParams()
    assert risk_level(0.0, p) == 0
    assert risk_level(SAFE_ANGLE + 1e-3, p) == 1
    assert risk_level(CAUTION_ANGLE + 1e-3, p) == 2
    assert risk_level(p.jackknife_angle + 1e-3, p) == 3


def test_time_to_jackknife_finite_for_reverse():
    p = TrailerParams()
    t = time_to_jackknife(-1.2, math.radians(6.0), 0.0, p)
    assert 5.0 < t < 30.0


def test_time_to_jackknife_forward_is_infinite():
    p = TrailerParams()
    t = time_to_jackknife(1.2, math.radians(6.0), 0.0, p)
    assert t == float("inf")


def test_time_to_jackknife_zero_when_already_folded():
    p = TrailerParams()
    assert time_to_jackknife(-1.0, 0.0, p.jackknife_angle, p) == 0.0


def test_max_safe_speed_decreases_with_steer():
    p = TrailerParams()
    phi = math.radians(20.0)
    slow = max_safe_reverse_speed(phi, math.radians(4.0), p)
    fast = max_safe_reverse_speed(phi, math.radians(10.0), p)
    assert fast < slow


def test_max_safe_speed_decreases_with_hitch_angle():
    p = TrailerParams()
    delta = math.radians(6.0)
    a = max_safe_reverse_speed(math.radians(10.0), delta, p)
    b = max_safe_reverse_speed(math.radians(50.0), delta, p)
    assert b < a


def test_guard_limits_speed_but_keeps_margin():
    p = TrailerParams()
    phi = math.radians(70.0)
    allowed, level, msg = guard_state(phi, math.radians(10.0), 1.2, p)
    assert allowed < 1.2
    assert level >= 2
    assert "限速" in msg


def test_guard_blocks_when_folded():
    p = TrailerParams()
    allowed, level, _msg = guard_state(p.jackknife_angle, 0.0, 1.0, p)
    assert allowed == 0.0
    assert level == 3


def test_params_reject_bad_values():
    with pytest.raises(ValueError):
        TrailerParams(tractor_wheelbase=0.0)
    with pytest.raises(ValueError):
        TrailerParams(max_steer=2.0)
    with pytest.raises(ValueError):
        TrailerParams(jackknife_angle=0.0)


def test_scenarios_expected_outcomes():
    results = {r.name: r for r in default_scenarios()}
    assert results["reverse_unguarded"].jackknifed is True
    assert results["reverse_guarded"].jackknifed is False
    assert results["reverse_guarded"].max_abs_phi_deg < 70.0
    assert results["forward_stable"].jackknifed is False


def test_guarded_scenario_never_exceeds_limit():
    p = TrailerParams()
    r = run_scenario("g", -1.5, math.radians(8.0), p, duration=60.0,
                     guarded=True)
    assert r.jackknifed is False
    assert r.max_abs_phi_deg < math.degrees(p.jackknife_angle)


def test_unguarded_scenario_hits_limit():
    p = TrailerParams()
    r = run_scenario("u", -1.5, math.radians(8.0), p, duration=60.0,
                     guarded=False)
    assert r.jackknifed is True
    assert r.max_abs_phi_deg >= math.degrees(p.jackknife_angle) - 1e-6
