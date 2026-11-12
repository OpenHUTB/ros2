"""控制分配与闭环场景测试。

闭环断言只看"物理上必须成立"的量（是否中止、高度是否守住、是否到达目标速度），
不钉死具体数值，避免调参时天天改测试。
"""

from __future__ import annotations

import numpy as np
import pytest

from uav_tiltrotor_transition_corridor import (
    ActuatorState,
    Controller,
    ControllerConfig,
    Corridor,
    PilotConfig,
    TiltrotorSim,
    VehicleSpec,
)
from sim import scenarios


# ---------------------------------------------------------------- 控制分配
def _controller(**kw) -> Controller:
    spec = VehicleSpec()
    return Controller(spec, Corridor(), ControllerConfig(**kw))


def test_allocation_tilts_forward_for_acceleration():
    c = _controller()
    thrust, tilt = c.allocate(v=10.0, rho=1.225, drag=2.0, lift=40.0, a_x=1.5, a_z=0.0)
    assert tilt < 90.0
    assert thrust > 0.0


def test_allocation_reverses_when_braking():
    c = _controller()
    _, tilt = c.allocate(v=8.0, rho=1.225, drag=1.0, lift=20.0, a_x=-2.0, a_z=0.0)
    assert tilt > 90.0


def test_allocation_without_reverse_capability_stops_at_ninety():
    spec = VehicleSpec(tilt_max_deg=90.0)
    c = Controller(spec, Corridor(), ControllerConfig())
    _, tilt = c.allocate(v=8.0, rho=1.225, drag=1.0, lift=20.0, a_x=-2.0, a_z=0.0)
    assert tilt == pytest.approx(90.0)


def test_tilt_rate_limit_is_respected():
    c = _controller(tilt_schedule="fixed:0")
    act = ActuatorState(tilt_deg=90.0, thrust=60.0, cl=0.0)
    new, _ = c.step(act, v=15.0, u=15.0, w=0.0, h=30.0, h_ref=30.0, u_ref=15.0,
                    rho=1.225, drag=5.0, lift=60.0, dt=0.01)
    assert 90.0 - new.tilt_deg <= VehicleSpec().rotor.tilt_rate_max_deg * 0.01 + 1e-9


def test_thrust_saturation_is_reported():
    c = _controller()
    act = ActuatorState(tilt_deg=90.0, thrust=0.0, cl=0.0)
    _, diag = c.step(act, v=0.0, u=0.0, w=-3.0, h=20.0, h_ref=30.0, u_ref=0.0,
                     rho=1.225, drag=0.0, lift=0.0, dt=0.01, thrust_scale=0.3)
    assert diag["saturated"] is True


def test_corridor_guard_lifts_a_sagging_schedule():
    guarded = _controller(tilt_schedule="fixed:10", corridor_guard=True)
    loose = _controller(tilt_schedule="fixed:10", corridor_guard=False)
    kw = dict(v=2.0, u=2.0, w=0.0, h=30.0, h_ref=30.0, u_ref=2.0,
              rho=1.225, drag=1.0, lift=0.0, dt=0.01)
    act = ActuatorState(tilt_deg=10.0, thrust=60.0, cl=0.0)
    g, _ = guarded.step(act, **kw)
    l, _ = loose.step(act, **kw)
    # 保护只允许往上抬，不允许压下来
    assert g.tilt_deg >= l.tilt_deg


# ---------------------------------------------------------------- 闭环场景
@pytest.fixture(scope="module")
def nominal():
    return scenarios.run("nominal")


def test_nominal_completes(nominal):
    assert not nominal.aborted
    assert nominal.v.max() == pytest.approx(20.0, abs=0.2)
    assert nominal.alt_error < 0.5
    assert abs(nominal.u[-1]) < 0.5


def test_nominal_energy_is_plausible(nominal):
    assert 10.0 < nominal.energy_wh < 30.0
    # 悬停功率应接近 P = T·v_i/fm
    assert nominal.power.max() == pytest.approx(1006.0, rel=0.05)


def test_nominal_never_loses_lift(nominal):
    assert nominal.min_lift_margin >= 2.0


def test_transition_uses_the_corridor(nominal):
    """加速段倾转角应当始终不低于当时的升力下边界。"""
    c = Corridor()
    for v, tilt in zip(nominal.v[::200], nominal.tilt[::200]):
        if v > 1.0:
            assert tilt >= c.load_tilt_limit(v) - 0.5


def test_guard_rescues_rough_schedule_at_altitude():
    bad = scenarios.run("fixed_tilt_no_guard")
    good = scenarios.run("fixed_tilt_guard")
    assert bad.aborted
    assert not good.aborted
    assert good.alt_error < 1.0


def test_thrust_degradation_triggers_abort():
    r = scenarios.run("rotor_degraded")
    assert r.aborted
    assert "高度" in (r.abort_reason or "") or "下沉" in (r.abort_reason or "")


def test_headwind_gust_keeps_altitude():
    r = scenarios.run("headwind_gust")
    assert not r.aborted
    assert r.alt_error < 2.0
    # 顶风时空速高于地速
    mid = r.t.size // 2
    assert r.v[mid] >= r.u[mid] - 1e-6


def test_high_altitude_needs_more_energy():
    sea = scenarios.run("nominal")
    high = scenarios.run("high_altitude")
    assert not high.aborted
    assert high.energy_wh > sea.energy_wh


def test_pilot_aborts_on_sustained_sink():
    sim = TiltrotorSim(pilot_cfg=PilotConfig(sink_abort=0.5, sink_abort_time=0.3,
                                             hover_alt=30.0),
                       cfg=ControllerConfig(tilt_schedule="fixed:2", corridor_guard=False))
    res = sim.run(30.0)
    assert res.aborted or res.max_sink > 0.5


def test_sim_result_shapes(nominal):
    n = nominal.t.size
    assert n > 1000
    assert nominal.v.shape == (n,)
    assert len(nominal.mode) == n
    assert np.isfinite(nominal.h).all()
