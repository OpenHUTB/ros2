import math

import numpy as np
import pytest

from quad_cascade_pid_hover_wind.dynamics import (QuadParams, Quadrotor, State,
                                                   WindParams, thrust_to_omega)


@pytest.fixture
def quad():
    return Quadrotor(QuadParams(), WindParams())


def test_hover_thrust_balances_gravity(quad):
    # 悬停转速应使总推力约等于重力
    omega = math.sqrt(quad.p.mass * quad.p.gravity / (4 * quad.p.k_f))
    s = State(omega_cmd=np.full(4, omega))
    force, moment = quad.forces_and_moments(s, np.zeros(3))
    # 世界系 z 方向合力应约为 0（推力向上平衡重力）
    assert abs(force[2]) < 0.5
    # 无风无姿态时力矩应为 0
    assert np.linalg.norm(moment) < 1e-6


def test_mixer_pure_yaw(quad):
    # 仅偏航力矩时，四个旋翼推力成对增减，总推力不变
    u = np.array([quad.p.mass * quad.p.gravity, 0.0, 0.0, 0.05])
    thrusts = quad._mixer @ u
    # 总推力应等于重力
    assert abs(np.sum(thrusts) - quad.p.mass * quad.p.gravity) < 1e-6
    # 两个 CCW 旋翼与两个 CW 旋翼推力不同
    assert thrusts[1] != thrusts[0]


def test_dynamics_conserves_orientation_norm(quad):
    s = State(pos=np.array([0.0, 0.0, 2.0]))
    hover = math.sqrt(quad.p.mass * quad.p.gravity / (4 * quad.p.k_f))
    s.omega_cmd = np.full(4, hover)
    for _ in range(100):
        s, _ = quad.step(s, np.full(4, hover), 0.002, 0.0)
    # 四元数模长始终为 1
    assert abs(np.linalg.norm(s.quat) - 1.0) < 1e-6
    # 悬停时高度基本不变
    assert abs(s.pos[2] - 2.0) < 0.1


def test_wind_gust_profile():
    w = WindParams(gust_start=5.0, gust_duration=4.0,
                   gust_magnitude=np.array([3.0, 0.0, 0.0]))
    q = Quadrotor(QuadParams(), w)
    v_before = q.wind_velocity(3.0, 0.01)
    v_peak = q.wind_velocity(7.0, 0.01)
    v_after = q.wind_velocity(12.0, 0.01)
    assert abs(v_peak[0]) > abs(v_before[0])
    # 阵风过后风速回到均值附近
    assert abs(v_after[0]) < 0.5


def test_thrust_to_omega_clips():
    omega = thrust_to_omega(np.array([-1.0, 1e6]), 3.2e-6, 200.0, 1100.0)
    assert omega[0] == 200.0
    assert omega[1] == 1100.0
