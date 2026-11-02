#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pid_altitude_controller 单元测试（不依赖 ROS）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import PID, AltitudePlant, G, simulate_altitude_step  # noqa: E402


def test_pid_proportional_only():
    pid = PID(kp=1.0, ki=0.0, kd=0.0, output_limit=10.0)
    assert abs(pid.update(10.0, 0.0, 0.02) - 10.0) < 1e-9


def test_pid_output_limit():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, output_limit=1.0)
    assert pid.update(10.0, 0.0, 0.02) == 1.0


def test_pid_integral_accumulates():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
    u1 = pid.update(1.0, 0.0, 0.1)
    u2 = pid.update(1.0, 0.0, 0.1)
    assert u2 > u1  # 积分项持续累加


def test_plant_hover():
    """节流阀恰好抵消重力时高度不变。"""
    plant = AltitudePlant(max_accel=G, z0=5.0)
    plant.step(1.0, 0.02)
    assert abs(plant.z - 5.0) < 1e-9


def test_plant_climb():
    plant = AltitudePlant(z0=0.0)  # 默认满推力 16 m/s^2 > g
    plant.step(1.0, 1.0)  # 满推力 1 秒
    assert plant.z > 0.0


def test_step_response_reaches_target():
    """良好参数下高度应收敛到目标值（稳态误差 < 5%）。"""
    ts, zs, us, m = simulate_altitude_step(kp=1.2, ki=0.6, kd=0.8,
                                           target_alt=10.0, duration=30.0)
    assert m['steady_error'] < 0.5


def test_overshoot_recorded():
    ts, zs, us, m = simulate_altitude_step(kp=3.0, ki=0.5, kd=0.2,
                                           target_alt=10.0, duration=30.0)
    # 高比例 kp 会产生超调
    assert m['overshoot_pct'] >= 0.0


def test_windup_guard():
    """抗积分饱和: 限幅期间积分不应无限增长。"""
    pid = PID(kp=0.5, ki=2.0, kd=0.0, output_limit=1.0)
    plant = AltitudePlant()
    for _ in range(500):
        pid.update(50.0, plant.z, 0.02)
    # 若无保护, 积分 = 50*10 = 500; 有保护应远小于该值
    assert abs(pid.integral) < 100.0
