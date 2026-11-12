# -*- coding: utf-8 -*-
"""故障算子单元测试：只依赖 numpy，不碰 MuJoCo，秒级跑完。"""

import numpy as np
import pytest

from uav_actuator_fault_sim import fault as F

TMAX = 6.0


def _inj(rotor_faults=(), sensor=None):
    return F.FaultInjector(F.FaultSpec(rotor_faults=tuple(rotor_faults),
                                       sensor=sensor or F.SensorFault()),
                           max_thrust=TMAX)


# --------------------------------------------------------------------------- #
# 单桨故障的四种形态
# --------------------------------------------------------------------------- #
def test_loss_before_onset_is_identity():
    inj = _inj([F.RotorFault(1, "loss", onset=2.0)])
    assert np.allclose(inj.gains(0.0), 1.0)
    assert np.allclose(inj.gains(1.99), 1.0)
    assert not inj.active(1.99)
    assert inj.active(2.0)


def test_loss_kills_exactly_one_rotor():
    inj = _inj([F.RotorFault(2, "loss", onset=1.0)])
    g = inj.gains(3.0)
    assert g[2] == 0.0
    assert np.allclose(np.delete(g, 2), 1.0)


def test_degradation_keeps_fraction():
    inj = _inj([F.RotorFault(0, "degradation", onset=1.0, severity=0.35)])
    assert inj.gains(2.0)[0] == pytest.approx(0.35)


def test_stuck_has_zero_gain_and_positive_offset():
    inj = _inj([F.RotorFault(3, "stuck", onset=1.0, severity=0.4)])
    g = inj.gains(2.0)
    b = inj.offsets(2.0)
    assert g[3] == 0.0
    assert b[3] > 0.0
    # 卡在 0.4·T_max 处：指令为 0 时推力仍为 b
    u = np.zeros(4)
    assert inj.apply(u, 2.0)[3] == pytest.approx(0.4 * TMAX, rel=1e-9)


def test_progressive_decays_monotonically_and_respects_floor():
    inj = _inj([F.RotorFault(1, "progressive", onset=1.0, severity=0.5)])
    g2, g3, g4 = (inj.gains(t)[1] for t in (2.0, 3.0, 20.0))
    assert 0.0 < g4 <= g3 < g2 < 1.0
    assert g4 == pytest.approx(0.05)  # 保留残余推力的下限


def test_ramp_spreads_the_transition():
    """ramp>0 时 transition 期间增益取中间值。"""
    inj = _inj([F.RotorFault(0, "loss", onset=1.0, ramp=2.0)])
    assert inj.gains(1.0)[0] == pytest.approx(1.0)
    assert inj.gains(2.0)[0] == pytest.approx(0.5)
    assert inj.gains(3.0)[0] == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# 注入器与多故障组合
# --------------------------------------------------------------------------- #
def test_apply_clips_to_physical_range():
    inj = _inj([F.RotorFault(0, "stuck", onset=0.0, severity=1.0)])
    applied = inj.apply(np.full(4, 2.0), 1.0)
    assert applied.shape == (4,)
    assert np.all(applied >= 0.0)
    assert np.all(applied <= TMAX + 1e-9)


def test_apply_preserves_healthy_rotors():
    inj = _inj([F.RotorFault(1, "loss", onset=1.0)])
    u = np.array([1.0, 2.0, 3.0, 4.0])
    out = inj.apply(u, 2.0)
    assert np.allclose(out[[0, 2, 3]], u[[0, 2, 3]])
    assert out[1] == 0.0


def test_two_lost_rotors_combine():
    inj = _inj([F.RotorFault(1, "loss", onset=1.0), F.RotorFault(3, "loss", onset=1.0)])
    g = inj.gains(2.0)
    assert g[1] == 0.0 and g[3] == 0.0
    assert np.allclose(g[[0, 2]], 1.0)


def test_earliest_onset_and_worst_case_gain():
    spec = F.FaultSpec(rotor_faults=(F.RotorFault(1, "loss", onset=3.0),
                                     F.RotorFault(2, "degradation", onset=1.0,
                                                  severity=0.6)))
    inj = F.FaultInjector(spec, max_thrust=TMAX)
    assert spec.earliest_onset == pytest.approx(1.0)
    assert inj.worst_case_gain() == pytest.approx(0.0)


def test_describe_is_human_readable():
    spec = F.FaultSpec(rotor_faults=(F.RotorFault(0, "loss", onset=2.0),))
    text = spec.describe()
    assert isinstance(text, str) and text
    assert "桨" in text or "rotor" in text.lower()


# --------------------------------------------------------------------------- #
# 传感器故障
# --------------------------------------------------------------------------- #
def test_sensor_fault_activity():
    quiet = F.SensorFault()
    assert not quiet.active
    noisy = F.SensorFault(gyro_bias=0.02, noise_scale=1.5, dropout=0.1)
    assert noisy.active


# --------------------------------------------------------------------------- #
# 工厂函数
# --------------------------------------------------------------------------- #
def test_factory_helpers_produce_expected_kinds():
    assert F.single_rotor_loss(1).rotor_faults[0].kind == "loss"
    assert F.dual_rotor_loss(0, 2).rotor_faults[1].rotor == 2
    assert F.partial_degradation(0, 0.5).rotor_faults[0].kind == "degradation"
    assert F.rotor_stuck(0, 0.3).rotor_faults[0].kind == "stuck"
    assert F.progressive_decay(0, 0.2).rotor_faults[0].kind == "progressive"
    assert F.sensor_gyro_bias(0.01).sensor.gyro_bias == pytest.approx(0.01)


def test_combine_merges_specs():
    a = F.single_rotor_loss(0)
    b = F.sensor_gyro_bias(0.02)
    merged = F.combine(a, b)
    assert len(merged.rotor_faults) == 1
    assert merged.sensor.gyro_bias == pytest.approx(0.02)


# --------------------------------------------------------------------------- #
# 参数校验
# --------------------------------------------------------------------------- #
def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        F.RotorFault(0, "not_a_kind")
    with pytest.raises(ValueError):
        F.RotorFault(4, "loss")          # 桨序号越界
    with pytest.raises(ValueError):
        F.RotorFault(0, "loss", ramp=-1.0)
    with pytest.raises(ValueError):
        F.RotorFault(0, "progressive", severity=0.0)
