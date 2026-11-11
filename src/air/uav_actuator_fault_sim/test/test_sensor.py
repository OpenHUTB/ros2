# -*- coding: utf-8 -*-
"""传感器故障的单元测试。

重点验证两件事：故障算子本身算得对，以及**故障确实进了控制回路**
（否则 ``sensor_bias`` 场景就等于 ``healthy``，那就成了死代码）。
"""

from __future__ import annotations

import numpy as np
import pytest

from uav_actuator_fault_sim import fault as F

MAX_THRUST = 6.0
TRUE_W = np.array([0.0, 0.0, 0.0])


class _AlwaysDrop:
    """把丢包概率顶成 1 的假随机源。"""

    def random(self) -> float:
        return 0.0

    def normal(self, loc=0.0, scale=1.0, size=None):
        return np.full(size if size is not None else 3, loc)


# --------------------------------------------------------------------------- #
# 故障算子
# --------------------------------------------------------------------------- #
def test_sense_omega_before_onset_is_truth():
    inj = F.FaultInjector(F.sensor_gyro_bias(0.1, onset=2.0), max_thrust=MAX_THRUST)
    out = inj.sense_omega(TRUE_W, 1.9)
    assert np.allclose(out, TRUE_W)


def test_sense_omega_disabled_is_truth():
    inj = F.FaultInjector(F.FaultSpec(), max_thrust=MAX_THRUST)
    assert inj.sensor_alpha(10.0) == 0.0
    assert np.allclose(inj.sense_omega([0.3, -0.2, 0.1], 10.0), [0.3, -0.2, 0.1])


def test_sense_omega_ramp_is_proportional():
    inj = F.FaultInjector(F.sensor_gyro_bias(0.2, onset=2.0), max_thrust=MAX_THRUST)
    # 默认 ramp = 0.5 s：2.25 s 时走到一半
    half = inj.sense_omega(TRUE_W, 2.25)
    assert 0.0 < float(half[0]) < 0.2
    assert abs(float(half[0]) - 0.1) < 1e-9


def test_sense_omega_bias_only_on_x_axis():
    inj = F.FaultInjector(F.sensor_gyro_bias(0.3, onset=0.0), max_thrust=MAX_THRUST)
    out = inj.sense_omega(TRUE_W, 1.0)
    assert abs(float(out[0]) - 0.3) < 1e-12
    assert abs(float(out[1])) < 1e-12 and abs(float(out[2])) < 1e-12


def test_sense_omega_dropout_holds_previous_sample():
    spec = F.FaultSpec(sensor=F.SensorFault(dropout=1.0, onset=0.0, ramp=0.0))
    inj = F.FaultInjector(spec, max_thrust=MAX_THRUST)
    prev = np.array([0.5, 0.6, 0.7])
    out = inj.sense_omega(TRUE_W, 1.0, rng=_AlwaysDrop(), prev=prev)
    assert np.allclose(out, prev)


def test_sense_omega_noise_scale_adds_noise():
    spec = F.FaultSpec(sensor=F.SensorFault(noise_scale=50.0, onset=0.0, ramp=0.0))
    inj = F.FaultInjector(spec, max_thrust=MAX_THRUST)
    rng = np.random.default_rng(0)
    outs = np.array([inj.sense_omega(TRUE_W, 1.0, rng=rng) for _ in range(20)])
    assert float(np.std(outs[:, 1])) > 0.0


def test_sensor_onset_enters_earliest_onset():
    spec = F.sensor_gyro_bias(0.1, onset=3.0)
    assert spec.earliest_onset == pytest.approx(3.0)
    assert F.FaultSpec().earliest_onset is None


@pytest.mark.parametrize("kwargs", [
    {"onset": -0.1},
    {"ramp": -0.1},
    {"dropout": 1.5},
    {"noise_scale": 0.0},
])
def test_sensor_fault_rejects_bad_params(kwargs):
    with pytest.raises(ValueError):
        F.SensorFault(**kwargs)


# --------------------------------------------------------------------------- #
# 接进仿真：同样没有旋翼故障，但传感器零偏要飞得不一样
# --------------------------------------------------------------------------- #
def _run(scenario: str):
    pytest.importorskip("mujoco")
    from uav_actuator_fault_sim._compat import load_base
    from uav_actuator_fault_sim.runner import RunConfig, run_case
    from uav_actuator_fault_sim.scenario import make_case

    try:
        api = load_base()
    except Exception as exc:            # noqa: BLE001 - 没有底座就跳过
        pytest.skip(f"缺少 uav_mujoco_sim 底座：{exc}")
    case = make_case("open_ground", scenario, seed=0)
    return run_case(case, RunConfig(duration=5.0), api=api)


def test_sensor_bias_changes_flight():
    healthy = _run("healthy")
    biased = _run("sensor_bias")
    assert not healthy.crashed
    assert biased.horizontal_drift > healthy.horizontal_drift + 0.05


def test_sensor_bias_is_recorded_in_result():
    biased = _run("sensor_bias")
    assert biased.omega_sensed is not None
    gap = float(np.max(np.abs(biased.omega_sensed - biased.omega)))
    assert gap > 0.05                      # 零偏 0.1 rad/s，过渡后应能看出来


def test_healthy_sensor_stream_equals_truth():
    healthy = _run("healthy")
    assert healthy.omega_sensed is not None
    assert np.allclose(healthy.omega_sensed, healthy.omega)


def test_sensor_bias_does_not_touch_rotor_health():
    biased = _run("sensor_bias")
    assert np.allclose(biased.health, 1.0)   # 旋翼全健康，只能靠姿态漂移看出来
