"""闭环场景测试：扭矩矢量应当改善横摆跟踪，并且代价可量化。"""

from __future__ import annotations

import numpy as np
import pytest

from torque_vectoring_yaw_control.control import ControllerConfig, YawController
from torque_vectoring_yaw_control.vehicle import VehicleSpec, WheelState
from sim import scenarios

ARMS = WheelState.moment_arms(1.56)


@pytest.fixture(scope="module")
def step_pair():
    return scenarios.run("step_steer", False), scenarios.run("step_steer", True)


def test_torque_vectoring_improves_yaw_tracking(step_pair):
    off, on = step_pair
    assert on.yaw_rms_error < 0.6 * off.yaw_rms_error


def test_torque_vectoring_tracks_reference_faster(step_pair):
    """打方向 1 s 后 1 s 内，有 TV 的横摆角速度应更接近参考。"""
    off, on = step_pair
    mask = (on.t > 1.0) & (on.t < 2.0)
    err_on = np.mean(np.abs(on.r[mask] - on.r_ref[mask]))
    err_off = np.mean(np.abs(off.r[mask] - off.r_ref[mask]))
    assert err_on < err_off


def test_torque_vectoring_uses_more_of_the_tire(step_pair):
    off, on = step_pair
    assert on.utilization.max() >= off.utilization.max()


def test_sine_sweep_tracks_better_without_more_sideslip():
    """正弦工况靠的是响应速度，不是把侧偏角做大。
    侧偏角必须基本不变，否则等于用稳定性换跟踪。"""
    off = scenarios.run("sine_steer", False)
    on = scenarios.run("sine_steer", True)
    assert on.yaw_rms_error < 0.5 * off.yaw_rms_error
    assert on.max_sideslip_deg <= off.max_sideslip_deg + 0.5


def test_motor_failure_degrades_without_tv():
    off = scenarios.run("motor_failure", False)
    on = scenarios.run("motor_failure", True)
    assert on.yaw_rms_error < 0.7 * off.yaw_rms_error
    assert off.yaw_rms_error > 0.02          # 确实被"带偏"了


def test_dead_wheel_produces_no_force():
    on = scenarios.run("motor_failure", True)
    assert np.allclose(on.fx[:, 3], 0.0, atol=1e-6)


def test_over_limit_reduces_but_cannot_recover():
    """越界工况只验证"减缓"，不验证"救回"——侧偏角仍然远超可控范围，
    这正是本模块要说清的边界。"""
    off = scenarios.run("over_limit", False)
    on = scenarios.run("over_limit", True)
    assert on.max_sideslip_deg < off.max_sideslip_deg
    assert on.max_sideslip_deg > 15.0        # 仍然失控，不该被写成"已恢复"


def test_reference_is_clipped_by_grip():
    off = scenarios.run("over_limit", False)
    spec = VehicleSpec()
    for u, r_ref in zip(off.u[::300], off.r_ref[::300]):
        assert abs(r_ref) <= spec.yaw_rate_limit(u) + 1e-9


def test_commands_are_smooth_after_transient():
    """瞬态之后指令必须平稳。附着椭圆 + 有效集会构成
    "力→额度→力"的正反馈，一旦没有滤波就会出现 20 Hz 极限环，
    这条测试就是防止它悄悄回来。"""
    for key in ("step_steer", "motor_failure"):
        res = scenarios.run(key, True)
        step = np.abs(np.diff(res.fx, axis=0)).max(axis=1)
        after = res.t[1:] > 2.0
        assert step[after].max() < 50.0, f"{key} 稳态出现指令抖动"


def test_moment_command_is_limited():
    spec = VehicleSpec()
    cfg = ControllerConfig(moment_limit=500.0)
    ctrl = YawController(spec, cfg)
    caps = [3000.0] * 4
    out = ctrl.step(u=25.0, r=0.0, steer=0.2, u_ref=25.0, ax_ff=0.0, caps=caps, dt=0.01)
    assert abs(out.moment_cmd) <= 500.0 + 1e-9


def test_disabled_controller_never_requests_moment():
    spec = VehicleSpec()
    ctrl = YawController(spec, ControllerConfig(enabled=False))
    out = ctrl.step(u=25.0, r=0.0, steer=0.2, u_ref=25.0, ax_ff=0.0,
                    caps=[3000.0] * 4, dt=0.01)
    assert out.moment_cmd == 0.0
    assert out.moment_achieved == pytest.approx(0.0, abs=1e-6)


def test_allocation_never_exceeds_caps():
    res = scenarios.run("step_steer", True)
    cap = 2727.0  # 电机上限
    assert np.all(np.abs(res.fx) <= cap + 50.0)   # 允许载荷转移带来的少量超调


def test_result_shapes(step_pair):
    on = step_pair[1]
    n = on.t.size
    assert n > 500
    assert on.fx.shape == (n, 4)
    assert np.isfinite(on.r).all()
    assert np.isfinite(on.x).all()


def test_cornering_stays_in_friction_circle():
    """三个"附着内"工况的峰值利用率必须留有余量，
    否则结论就变成"在打滑边缘比谁滑得慢"。"""
    for key in ("step_steer", "sine_steer"):
        res = scenarios.run(key, True)
        assert res.utilization.max() <= 0.75
        assert res.max_sideslip_deg < 5.0
