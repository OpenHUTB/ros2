#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""无人机起飞—悬停—降落状态机（纯 Python 实现，不依赖 ROS）。

该模块只负责“给定当前位置和目标，算出速度指令”，不订阅话题、不发布消息，
因此可以脱离 ROS 单独做单元测试，也方便在离线仿真中复用。

状态流转：

    IDLE ──首帧位置──▶ TAKEOFF ──到达起飞高度──▶ HOVER
                                                  │ 悬停时间到
                                                  ▼
                                                LAND ──触地──▶ DONE

坐标系约定与仓库其他空域模块一致：ENU（东-北-天），z 向上为正；
速度指令为机体系（x 前、y 左、z 上），本实验假设偏航角保持 0，
所以机体系与 ENU 方向一致。
"""
from __future__ import print_function

STATE_IDLE = 'IDLE'
STATE_TAKEOFF = 'TAKEOFF'
STATE_HOVER = 'HOVER'
STATE_LAND = 'LAND'
STATE_DONE = 'DONE'

STATE_NAMES = {
    STATE_IDLE: '待机',
    STATE_TAKEOFF: '起飞',
    STATE_HOVER: '悬停',
    STATE_LAND: '降落',
    STATE_DONE: '完成',
}


def _clamp(value, limit):
    """把 value 限制在 [-limit, limit] 内。"""
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class Command(object):
    """状态机输出的速度指令（机体系，m/s 与 rad/s）。"""

    __slots__ = ('vx', 'vy', 'vz', 'yaw_rate', 'state')

    def __init__(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, state=STATE_IDLE):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw_rate = yaw_rate
        self.state = state

    def __repr__(self):
        return ('Command(vx=%.3f, vy=%.3f, vz=%.3f, yaw_rate=%.3f, state=%s)'
                % (self.vx, self.vy, self.vz, self.yaw_rate, self.state))


class TakeoffHoverLandFSM(object):
    """起飞—悬停—降落状态机。

    参数
    ----
    takeoff_altitude : 起飞目标高度（m，相对地面）
    hover_time       : 到点后悬停保持时间（s）
    position_tolerance : 判定“到点”的位置误差（m）
    max_horizontal_speed / max_vertical_speed : 速度限幅（m/s）
    kp_horizontal / kp_vertical : 位置误差到速度指令的比例系数（1/s）
    land_altitude    : 判定触地的高度（m）
    yaw_rate         : 期望偏航角速度，默认 0（保持机头方向）
    """

    def __init__(self, takeoff_altitude=2.0, hover_time=5.0,
                 position_tolerance=0.15, max_horizontal_speed=1.0,
                 max_vertical_speed=0.5, kp_horizontal=0.8,
                 kp_vertical=1.0, land_altitude=0.05, yaw_rate=0.0,
                 hover_offset=(0.0, 0.0)):
        self.takeoff_altitude = float(takeoff_altitude)
        self.hover_time = float(hover_time)
        self.position_tolerance = float(position_tolerance)
        self.max_horizontal_speed = float(max_horizontal_speed)
        self.max_vertical_speed = float(max_vertical_speed)
        self.kp_horizontal = float(kp_horizontal)
        self.kp_vertical = float(kp_vertical)
        self.land_altitude = float(land_altitude)
        self.yaw_rate = float(yaw_rate)
        self.hover_offset = (float(hover_offset[0]), float(hover_offset[1]))

        self.state = STATE_IDLE
        self.origin = None          # 起飞点 (x, y)
        self.hover_target = None    # 悬停目标 (x, y, z)
        self.hover_elapsed = 0.0    # 已悬停时间
        self.elapsed = 0.0          # 状态机运行总时间
        self.max_altitude = 0.0     # 记录到达过的最高高度

    def reset(self):
        """恢复到初始状态，便于重复运行。"""
        self.state = STATE_IDLE
        self.origin = None
        self.hover_target = None
        self.hover_elapsed = 0.0
        self.elapsed = 0.0
        self.max_altitude = 0.0

    def _track(self, target, position):
        """位置误差 -> 速度指令（比例控制 + 限幅）。"""
        ex = target[0] - position[0]
        ey = target[1] - position[1]
        ez = target[2] - position[2]
        vx = _clamp(self.kp_horizontal * ex, self.max_horizontal_speed)
        vy = _clamp(self.kp_horizontal * ey, self.max_horizontal_speed)
        vz = _clamp(self.kp_vertical * ez, self.max_vertical_speed)
        return vx, vy, vz

    def update(self, position, dt):
        """输入当前位置 (x, y, z) 与时间步长 dt，返回 Command。"""
        if dt <= 0.0:
            raise ValueError('dt must be positive')

        x, y, z = position
        self.elapsed += dt
        if z > self.max_altitude:
            self.max_altitude = z

        if self.state == STATE_IDLE:
            # 第一帧位置就是起飞点；悬停点 = 起飞点 + 水平偏移 + 目标高度
            self.origin = (x, y)
            self.hover_target = (x + self.hover_offset[0],
                                 y + self.hover_offset[1],
                                 self.takeoff_altitude)
            self.state = STATE_TAKEOFF

        if self.state == STATE_TAKEOFF:
            vx, vy, vz = self._track(self.hover_target, position)
            dx = self.hover_target[0] - x
            dy = self.hover_target[1] - y
            dz = self.hover_target[2] - z
            distance = (dx * dx + dy * dy + dz * dz) ** 0.5
            if distance <= self.position_tolerance:
                self.state = STATE_HOVER
                self.hover_elapsed = 0.0
            return Command(vx, vy, vz, self.yaw_rate, self.state)

        if self.state == STATE_HOVER:
            self.hover_elapsed += dt
            vx, vy, vz = self._track(self.hover_target, position)
            if self.hover_elapsed >= self.hover_time:
                self.state = STATE_LAND
            return Command(vx, vy, vz, self.yaw_rate, self.state)

        if self.state == STATE_LAND:
            # 降落阶段只保持水平位置，垂直方向匀速下降
            vx = _clamp(self.kp_horizontal * (self.hover_target[0] - x),
                        self.max_horizontal_speed)
            vy = _clamp(self.kp_horizontal * (self.hover_target[1] - y),
                        self.max_horizontal_speed)
            vz = -self.max_vertical_speed
            if z <= self.land_altitude:
                self.state = STATE_DONE
                return Command(0.0, 0.0, 0.0, self.yaw_rate, self.state)
            return Command(vx, vy, vz, self.yaw_rate, self.state)

        # DONE 或未知状态：保持静止
        return Command(0.0, 0.0, 0.0, self.yaw_rate, self.state)
