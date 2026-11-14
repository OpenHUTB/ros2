#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人形机器人运动学模型
状态: x, y, theta(朝向), v(速度), gait_phase(步态相位)
输入: forward(前后速度), turn(转向角速度)
"""
import math


class HumanoidModel:
    def __init__(self, height=1.7, max_speed=1.2, max_turn=1.0,
                 leg_length=0.85, arm_length=0.6, dt=0.02):
        self.height = height
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.leg = leg_length
        self.arm = arm_length
        self.dt = dt
        self.reset()

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.omega = 0.0
        self.gait_phase = 0.0

    def step(self, forward_cmd, turn_cmd):
        self.v = max(-self.max_speed, min(self.max_speed, forward_cmd))
        self.omega = max(-self.max_turn, min(self.max_turn, turn_cmd))
        self.theta += self.omega * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        gait_freq = 2.0 * abs(self.v) / 1.0
        self.gait_phase += gait_freq * 2.0 * math.pi * self.dt
        self.gait_phase = self.gait_phase % (2.0 * math.pi)

    @property
    def state(self):
        return (self.x, self.y, self.theta, self.v)

    def leg_angles(self):
        if abs(self.v) < 0.05:
            return (0.0, 0.0)
        swing = math.sin(self.gait_phase) * 0.4 * min(1.0, abs(self.v) / 0.8)
        return (swing, -swing)

    def arm_angles(self):
        if abs(self.v) < 0.05:
            return (0.0, 0.0)
        swing = math.sin(self.gait_phase + math.pi) * 0.3 * min(1.0, abs(self.v) / 0.8)
        return (swing, -swing)
