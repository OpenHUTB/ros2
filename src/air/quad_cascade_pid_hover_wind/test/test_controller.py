import math

import numpy as np
import pytest

from quad_cascade_pid_hover_wind.controller import (CascadeController,
                                                      ControllerGains, PIDGains,
                                                      WaypointManager,
                                                      build_gains_from_dict)
from quad_cascade_pid_hover_wind.dynamics import QuadParams, Quadrotor, WindParams


@pytest.fixture
def setup():
    cfg = {
        "position_pid": {"kp": [2.0, 2.0, 3.5], "ki": [0.0, 0.0, 0.4],
                         "kd": [2.5, 2.5, 4.0], "i_max": [0.5, 0.5, 1.0],
                         "out_max": [4.0, 4.0, 6.0]},
        "attitude_pid": {"kp": [12.0, 12.0, 8.0], "ki": [0.5, 0.5, 0.3],
                         "kd": [0, 0, 0], "i_max": [0.5, 0.5, 0.5],
                         "out_max": [4.0, 4.0, 3.0]},
        "rate_pid": {"kp": [0.18, 0.18, 0.10], "ki": [0, 0, 0],
                     "kd": [0, 0, 0], "i_max": [0.05, 0.05, 0.05],
                     "out_max": [1.5, 1.5, 0.8]},
    }
    p = QuadParams()
    quad = Quadrotor(p, WindParams())
    gains = build_gains_from_dict(cfg)
    ctrl = CascadeController(p, gains, quad._mixer)
    return p, ctrl


def test_pid_zero_error_zero_integral():
    g = PIDGains(np.array([1.0, 1.0, 1.0]), np.array([0.1, 0.1, 0.1]),
                 np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 1.0]),
                 np.array([10.0, 10.0, 10.0]))
    from quad_cascade_pid_hover_wind.controller import PID
    pid = PID(g)
    out = pid.update(np.zeros(3), 0.01)
    assert np.allclose(out, 0.0)


def test_waypoint_manager_switch():
    wps = [[0, 0, 1], [1, 0, 1]]
    mgr = WaypointManager(wps, acceptance=0.2, hover_time=1.0)
    # 在第一个航点悬停 1 秒后切换
    pos = np.array([0.0, 0.0, 1.0])
    target, switched = mgr.update(pos, 0.0)
    assert np.allclose(target, [0, 0, 1])
    mgr.update(pos, 0.5)
    target, switched = mgr.update(pos, 0.5)
    assert switched
    assert np.allclose(target, [1, 0, 1])


def test_controller_produces_finite_motor_commands(setup):
    p, ctrl = setup
    pos = np.array([0.0, 0.0, 1.0])
    vel = np.zeros(3)
    quat = np.array([1.0, 0, 0, 0])
    omega = np.zeros(3)
    target = np.array([0.0, 0.0, 1.5])
    cmd = ctrl.compute(pos, vel, quat, omega, target, 0.01)
    assert cmd.shape == (4,)
    assert np.all(np.isfinite(cmd))
    assert np.all(cmd >= p.min_omega - 1e-6)
    assert np.all(cmd <= p.max_omega + 1e-6)


def test_closed_loop_hover_converges(setup):
    """纯 Python 闭环：从地面起飞到 1.5m 并悬停，误差应收敛。"""
    from quad_cascade_pid_hover_wind.dynamics import Quadrotor, State, WindParams
    p, ctrl = setup
    quad = Quadrotor(p, WindParams())
    s = State(pos=np.array([0.0, 0.0, 0.0]),
               quat=np.array([1.0, 0, 0, 0]))
    s.omega_cmd = np.full(4, math.sqrt(p.mass * p.gravity / (4 * p.k_f)))
    target = np.array([0.0, 0.0, 1.5])
    for _ in range(int(8.0 / 0.002)):
        cmd = ctrl.compute(s.pos, s.vel, s.quat, s.omega, target, 0.002)
        s, _ = quad.step(s, cmd, 0.002, 0.0)
    err = np.linalg.norm(s.pos - target)
    assert err < 0.15  # 8 秒内收敛到 15cm 以内


def test_desired_attitude_uses_exact_atan2(setup):
    """期望姿态应由 atan2 精确解算，而不是小角度近似。"""
    p, ctrl = setup
    a_des = np.array([2.0, -1.0, 1.0])
    yaw = 0.3
    phi, theta, psi = ctrl._desired_attitude(a_des, yaw)
    fz = a_des[2] + p.gravity
    assert math.isclose(phi, math.atan2(2.0 * math.sin(yaw) + math.cos(yaw), fz),
                        rel_tol=1e-9)
    assert math.isclose(theta, math.atan2(2.0 * math.cos(yaw) - math.sin(yaw), fz),
                        rel_tol=1e-9)
    assert math.isclose(psi, yaw)


def test_thrust_reallocation_reduces_wrench_error(setup):
    """大力矩需求导致部分旋翼限幅时，重分配应保持总推力并降低 wrench 偏差。"""
    p, ctrl = setup
    u = np.array([p.mass * p.gravity, 1.4, 1.4, 0.0])  # 接近 rate 环限幅的大力矩
    raw = ctrl.mixer @ u
    max_thrust = p.k_f * p.max_omega ** 2
    clipped = np.clip(raw, 0.0, max_thrust)
    realloc = ctrl._reallocate(clipped, u)
    alloc_pinv = np.linalg.pinv(ctrl.mixer)
    err_clipped = np.linalg.norm(alloc_pinv @ clipped - u)
    err_realloc = np.linalg.norm(alloc_pinv @ realloc - u)
    assert np.all(realloc >= -1e-9) and np.all(realloc <= max_thrust + 1e-9)
    # 总推力保持
    assert math.isclose(float(np.sum(realloc)), u[0], rel_tol=1e-6)
    assert err_realloc < err_clipped
