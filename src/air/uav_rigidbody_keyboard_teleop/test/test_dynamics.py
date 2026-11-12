#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""刚体动力学单元测试。"""

import numpy as np

from uav_rigidbody_keyboard_teleop.dynamics import (
    Quadrotor,
    QuadParams,
    State,
    normalize_angle,
    validate_params,
)


def test_default_params_valid():
    assert validate_params(QuadParams()) == []


def test_hover_thrust_equals_weight():
    robot = Quadrotor(QuadParams())
    p = QuadParams()
    assert np.isclose(robot.hover_thrust(), p.mass * p.gravity)


def test_gravity_net_force():
    """旋翼零推力时，世界系合力应等于整机重力（-z）。"""
    params = QuadParams(min_omega=0.0)  # 关闭怠速残余推力
    robot = Quadrotor(params)
    state = State()
    state.pos[2] = 1.0
    force_world, _ = robot.forces_and_moments(state)
    assert np.isclose(force_world[2], -params.mass * params.gravity)


ZERO_WRENCH = {"thrust": 0.0, "tau_x": 0.0, "tau_y": 0.0, "tau_z": 0.0}


def test_free_fall_moves_down():
    """零推力时机体应向下运动。"""
    robot = Quadrotor(QuadParams())
    state = State()
    state.pos[2] = 1.0
    state = robot.step(state, ZERO_WRENCH, dt=0.01)
    assert state.vel[2] < 0.0


def test_ground_constraint():
    """落地后位置与垂直速度不应穿地。"""
    robot = Quadrotor(QuadParams())
    state = State()
    for _ in range(200):
        state = robot.step(state, ZERO_WRENCH, dt=0.01)
    assert state.pos[2] == 0.0
    assert state.vel[2] == 0.0


def test_angle_normalize():
    # 角度按模 2π 等价比较：π 与 -π 表示同一方向
    def same_angle(a, b):
        diff = abs((a - b + np.pi) % (2 * np.pi) - np.pi)
        return np.isclose(diff, 0.0)

    assert same_angle(normalize_angle(3 * np.pi), np.pi)
    assert same_angle(normalize_angle(-3 * np.pi), -np.pi)
    assert np.isclose(normalize_angle(0.5), 0.5)


def test_step_state_copy():
    """step 不应修改输入状态对象。"""
    robot = Quadrotor(QuadParams())
    state = State()
    before = state.pos.copy()
    robot.step(state, ZERO_WRENCH, dt=0.01)
    assert np.allclose(state.pos, before)
