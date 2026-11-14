#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼自行车运动学模型 (纯 Python, 不依赖 ROS)

状态: x, y, theta(航向), v(线速度)
输入: delta(前轮转角 rad), accel(加速度 m/s^2)

    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = v / L * tan(delta)
    v_dot     = accel
"""

import math


class AckermannBike:
    def __init__(self,
                 wheelbase=2.5,      # 轴距 L (m)
                 max_steer=0.5,      # 最大前轮转角 (rad) ~28.6°
                 max_speed=5.0,      # 最大线速度 (m/s)
                 max_accel=2.0,      # 最大加速度 (m/s^2)
                 dt=0.02):           # 默认步长 (s)
        self.L = wheelbase
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.dt = dt
        self.reset()

    def reset(self, x=0.0, y=0.0, theta=0.0, v=0.0):
        self.x, self.y, self.theta, self.v = x, y, theta, v

    def step(self, delta, accel):
        delta = max(-self.max_steer, min(self.max_steer, float(delta)))
        accel = max(-self.max_accel, min(self.max_accel, float(accel)))

        self.v += accel * self.dt
        self.v = max(-self.max_speed, min(self.max_speed, self.v))

        self.theta += self.v / self.L * math.tan(delta) * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt

    @property
    def state(self):
        return (self.x, self.y, self.theta, self.v)
