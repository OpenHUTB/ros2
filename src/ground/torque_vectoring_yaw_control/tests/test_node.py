"""节点层可离线验证的部分：轮位解析、附着额度、外部力需求直通。

``TorqueVectoringNode`` 本身要 rclpy 才能实例化，所以把真正有逻辑的部分
（参数解析、额度计算、力需求入口）拆成纯函数，在这里覆盖。
"""

from __future__ import annotations

import pytest

from torque_vectoring_yaw_control.control import ControllerConfig, YawController
from torque_vectoring_yaw_control.node import build, parse_dead_wheels
from torque_vectoring_yaw_control.vehicle import VehicleSpec, WheelState

ARMS = WheelState.moment_arms(1.56)


@pytest.mark.parametrize("text,expect", [
    ("3", (3,)),
    ("0,2", (0, 2)),
    (" 1 ; 3 ", (1, 3)),
    ("", ()),
    ("5,-1", ()),          # 越界直接丢弃
    ("abc,2", (2,)),       # 非数字跳过
    ("2,2", (2,)),         # 去重
])
def test_parse_dead_wheels(text, expect):
    assert parse_dead_wheels(text) == expect


def test_build_respects_mu_and_flag():
    spec, ctrl = build(enable_tv=False, mu=0.4)
    assert spec.mu == pytest.approx(0.4)
    assert ctrl.cfg.enabled is False
    # 低附着下横摆率上限必须随之收紧
    assert spec.yaw_rate_limit(30.0) == pytest.approx(0.4 * 9.80665 / 30.0, rel=1e-9)


def test_axle_forces_match_wheel_caps():
    """节点的额度 = min(附着椭圆, 电机能力)，两者都不能被突破。"""
    spec = VehicleSpec()
    fy_f, fy_r, fz_f, fz_r = spec.axle_forces(u=30.0, v=0.0, r=0.1, steer=0.05)
    for fy_axle, fz_axle in ((fy_f, fz_f), (fy_r, fz_r)):
        cap = min(spec.force_cap(fz_axle / 2, fy_axle / 2), spec.motor_force_max())
        assert 0.0 <= cap <= spec.motor_force_max() + 1e-6
        assert cap <= spec.mu * fz_axle / 2 + 1e-6


def test_axle_forces_clamp_at_grip_limit():
    """侧偏角足够大时侧向力必须停在 μFz，不会随角度无限增长。"""
    spec = VehicleSpec()
    _, _, fz_f, _ = spec.axle_forces(30.0, 0.0, 0.0, 0.0)
    fy_small = spec.axle_forces(30.0, 0.0, 0.0, 0.01)[0]
    fy_large = spec.axle_forces(30.0, 0.0, 0.0, 0.6)[0]
    assert fy_large == pytest.approx(spec.mu * fz_f, rel=1e-9)
    assert fy_small < fy_large


def test_external_force_demand_bypasses_speed_loop():
    """给定 force_demand 时，输出总力应当等于它（额度充足时），
    与 u/u_ref 的偏差无关——节点不该自己闭环车速。"""
    spec = VehicleSpec()
    ctrl = YawController(spec, ControllerConfig())
    caps = [spec.motor_force_max()] * 4
    out = ctrl.step(u=10.0, r=0.0, steer=0.0, u_ref=40.0, ax_ff=0.0,
                    caps=caps, dt=0.01, force_demand=1200.0)
    assert out.force_demand == pytest.approx(1200.0)
    assert sum(out.fx) == pytest.approx(1200.0, rel=1e-6)


def test_external_demand_is_split_evenly_when_moment_is_zero():
    spec = VehicleSpec()
    ctrl = YawController(spec, ControllerConfig())
    out = ctrl.step(u=25.0, r=0.0, steer=0.0, u_ref=0.0, ax_ff=0.0,
                    caps=[3000.0] * 4, dt=0.01, force_demand=1200.0)
    for f in out.fx:
        assert f == pytest.approx(300.0, rel=1e-6)


def test_dead_wheel_gets_zero_force():
    """失效轮额度为 0 时，分配器必须把它的出力压到 0。"""
    spec = VehicleSpec()
    ctrl = YawController(spec, ControllerConfig())
    caps = [spec.motor_force_max()] * 4
    caps[3] = 0.0
    out = ctrl.step(u=30.0, r=0.0, steer=0.05, u_ref=30.0, ax_ff=0.0,
                    caps=caps, dt=0.01, force_demand=1500.0)
    assert abs(out.fx[3]) < 1e-9
    assert sum(out.fx) == pytest.approx(1500.0, rel=1e-3)
