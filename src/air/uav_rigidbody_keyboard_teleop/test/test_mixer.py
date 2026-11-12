#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""混合分配矩阵单元测试。"""

import numpy as np

from uav_rigidbody_keyboard_teleop.dynamics import Quadrotor, QuadParams


def test_hover_thrust_distributes_equally():
    """悬停总推力应平均分配到四个旋翼。"""
    robot = Quadrotor(QuadParams())
    mg = robot.hover_thrust()
    thrusts = robot.allocate(mg, 0.0, 0.0, 0.0)
    assert np.allclose(thrusts, mg / 4.0, atol=1e-9)


def test_mixer_roundtrip():
    """逆混合矩阵与正向映射应互为逆映射。"""
    robot = Quadrotor(QuadParams())
    # 正向映射 A：wrench = A · thrusts；逆映射 M = A^{-1}
    # 校验 M @ A = I
    p = robot.p
    L = p.arm
    c = p.k_tau / p.k_f
    A = np.array([
        [1.0, 1.0, 1.0, 1.0],
        L * np.array([-1.0, 1.0, 1.0, -1.0]),
        L * np.array([-1.0, -1.0, 1.0, 1.0]),
        c * np.array([-1.0, 1.0, -1.0, 1.0]),
    ])
    M = robot._mixer
    assert np.allclose(M @ A, np.eye(4), atol=1e-9)


def test_yaw_moment_increases_ccw_thrust():
    """正偏航力矩应增大逆时针旋翼（FL/RR）推力。"""
    robot = Quadrotor(QuadParams())
    thrusts = robot.allocate(robot.hover_thrust(), 0.0, 0.0, 1.0)
    # SPIN_DIR: FR -1, FL +1, RL -1, RR +1
    ccw = thrusts[[1, 3]].mean()
    cw = thrusts[[0, 2]].mean()
    assert ccw > cw
