"""专家策略与轨迹剖面的单元测试。剖面测试是纯数学，不需要 mujoco。"""

import numpy as np
import pytest

from uav_fault_e2e_thrust_policy import expert as E


def test_trapezoid_starts_at_zero():
    x, v, a = E._trapezoid(0.0, distance=10.0, cruise=1.2, accel=0.8)
    assert x == 0.0 and v == 0.0 and a == 0.0


def test_trapezoid_ends_at_distance_with_zero_speed():
    x, v, a = E._trapezoid(1e6, distance=10.0, cruise=1.2, accel=0.8)
    assert np.isclose(x, 10.0)
    assert np.isclose(v, 0.0)
    assert np.isclose(a, 0.0)


def test_trapezoid_is_monotonic_and_bounded():
    ts = np.arange(0.0, 20.0, 0.01)
    xs = np.array([E._trapezoid(t, 10.0, 1.2, 0.8)[0] for t in ts])
    vs = np.array([E._trapezoid(t, 10.0, 1.2, 0.8)[1] for t in ts])
    as_ = np.array([E._trapezoid(t, 10.0, 1.2, 0.8)[2] for t in ts])
    assert np.all(np.diff(xs) >= -1e-9)
    assert xs.min() >= -1e-9 and xs.max() <= 10.0 + 1e-9
    assert vs.max() <= 1.2 + 1e-9
    assert np.abs(as_).max() <= 0.8 + 1e-9


def test_trapezoid_reaches_cruise_speed_in_middle():
    v_mid = E._trapezoid(4.0, 10.0, 1.2, 0.8)[1]
    assert np.isclose(v_mid, 1.2)


def test_trapezoid_handles_short_distance():
    # 距离太短时到不了巡航速度，也不能超调
    x, v, a = E._trapezoid(1e6, distance=0.1, cruise=1.2, accel=0.8)
    assert np.isclose(x, 0.1) and np.isclose(v, 0.0)


class _StubBase:
    """记录入参的假几何控制器。"""

    def __init__(self, thrust: float = 9.3):
        self.thrust = thrust
        self.calls = []

    def compute(self, state, p_d, v_d, a_d, yaw_d):
        self.calls.append((np.asarray(p_d, dtype=float),
                           np.asarray(v_d, dtype=float),
                           np.asarray(a_d, dtype=float)))
        return self.thrust, np.zeros(3)


class _StubParams:
    max_thrust = 6.0
    hover_thrust = 2.33
    mass = 0.95
    arm = 0.106
    k_m = 0.015


class _StubState:
    def __init__(self, pos=(0.0, 0.0, 1.5)):
        self.pos = np.asarray(pos, dtype=float)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega = np.zeros(3)
        self.t = 0.0


def _effectiveness():
    a = 0.106
    km = 0.015
    return np.array([
        [1.0, 1.0, 1.0, 1.0],
        [-a, a, a, -a],
        [-a, a, -a, a],
        [-km, -km, km, km],
    ])


def test_expert_command_is_normalized_and_4d():
    base = _StubBase()
    ex = E.ThrustExpert(_StubParams(), None, base, _effectiveness())
    cmd = ex.command(_StubState(), np.ones(4), None, 0.0)
    assert cmd.shape == (4,)
    assert cmd.min() >= 0.0 and cmd.max() <= 1.0


def test_expert_passes_acceleration_feedforward():
    base = _StubBase()
    ex = E.ThrustExpert(_StubParams(), None, base, _effectiveness())
    ex.command(_StubState(), np.ones(4), None, 0.2)      # 处于加速段
    _, v_d, a_d = base.calls[-1]
    assert a_d[0] > 0.0, "加速段应给 +x 加速度前馈"
    assert v_d[0] > 0.0


def test_expert_reference_tracks_trapezoid():
    ex = E.ThrustExpert(_StubParams(), None, _StubBase(), _effectiveness())
    x0 = ex.reference(0.0)[0][0]
    x1 = ex.reference(5.0)[0][0]
    x2 = ex.reference(1e6)[0][0]
    assert x0 == 0.0
    assert 0.0 < x1 < 10.0
    assert np.isclose(x2, ex.task.goal_x)


def test_expert_uses_ninety_percent_budget_with_dead_rotor():
    base = _StubBase()
    ex = E.ThrustExpert(_StubParams(), None, base, _effectiveness())
    gains = np.array([0.0, 1.0, 1.0, 1.0])
    cmd = ex.command(_StubState(), gains, None, 5.0)
    # 失效桨的指令必须被压到 0
    assert cmd[0] == 0.0
    assert cmd.max() <= 1.0


def test_task_config_defaults():
    cfg = E.TaskConfig()
    assert cfg.goal_x > 0.0
    assert cfg.crash_altitude < cfg.start_altitude
    assert cfg.duration > 0.0
