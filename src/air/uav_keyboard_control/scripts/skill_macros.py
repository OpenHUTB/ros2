# -*- coding: utf-8 -*-
"""键盘"技能宏"状态机（纯逻辑，不依赖 ROS / 终端，可直接单测）.

按键 1/2/3 各触发一个一次性动作，动作结束后自动回到悬停：

    1  起飞并升到 ``macro/takeoff_altitude``（默认 1.5 m）后悬停
    2  原地旋转 ``macro/spin_degrees``（默认 360°）
    3  沿当前机头方向前进 ``macro/forward_distance``（默认 3 m）并自动刹停

三个动作都靠 ``/uav/odom`` 的位姿闭环，而不是"开环数秒"：

    * 高度：P 律 + 死区，进入死区后连续 ``settle_ticks`` 帧才判定完成；
    * 旋转：按累计偏航（wrap 到 ±π）判断转够了没有，临近目标时降速；
    * 前进：按起点到当前点的投影距离算剩余里程，速度取 ``min(v_max, sqrt(2as))``，
      到点自动收油——和手动键一样，最终都只是一条机体系速度指令。

设计约束（与 keyboard_control.py 的分工）：

    * 只输出机体系速度 ``(vx, vy, vz, wz)``，不直接碰仿真器；
    * 任一方向键都会取消当前宏（手动优先）；
    * 位姿缺失或超时由调用方负责取消，本模块不判断时间。
"""
from __future__ import annotations

import math
from collections import namedtuple

# 位姿：位置 ENU（m）+ 偏航（rad，逆时针为正）
Pose = namedtuple("Pose", "x y z yaw")

ZERO_CMD = (0.0, 0.0, 0.0, 0.0)

MACRO_KEYS = ("1", "2", "3")
MACRO_NAMES = {"1": "TAKEOFF", "2": "SPIN", "3": "FORWARD"}


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_pi(angle):
    """把角度折到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(x, y, z, w):
    """四元数（x, y, z, w）-> 偏航角（rad）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Takeoff(object):
    """升到指定高度后悬停（P 律 + 死区 + 稳定帧计数）."""

    name = "TAKEOFF"

    def __init__(self, target_z=1.5, kp=2.0, max_speed=1.0,
                 tolerance=0.05, settle_ticks=6):
        self.target_z = float(target_z)
        self.kp = float(kp)
        self.max_speed = float(max_speed)
        self.tolerance = float(tolerance)
        self.settle_ticks = int(settle_ticks)
        self._hold = 0
        self.done = False

    def update(self, pose, dt=0.0):
        if self.done:
            return ZERO_CMD
        error = self.target_z - pose.z
        if abs(error) <= self.tolerance:
            self._hold += 1
            if self._hold >= self.settle_ticks:
                self.done = True
                return ZERO_CMD
        else:
            self._hold = 0
        return (0.0, 0.0, clamp(self.kp * error, self.max_speed), 0.0)


class Spin(object):
    """原地旋转指定角度（累计偏航，带 wrap，临近目标降速）."""

    name = "SPIN"

    def __init__(self, degrees=360.0, rate=1.0, tolerance_deg=2.0, min_rate=0.25):
        self.total = abs(math.radians(float(degrees)))
        self.sign = 1.0 if float(degrees) >= 0 else -1.0
        self.rate = float(rate)
        self.tolerance = math.radians(float(tolerance_deg))
        self.min_rate = float(min_rate)
        self.accumulated = 0.0
        self._last_yaw = None
        self.done = False

    def update(self, pose, dt=0.0):
        if self.done:
            return ZERO_CMD
        if self._last_yaw is None:
            self._last_yaw = pose.yaw
        self.accumulated += wrap_pi(pose.yaw - self._last_yaw)
        self._last_yaw = pose.yaw

        remaining = self.total - abs(self.accumulated)
        if remaining <= self.tolerance:
            self.done = True
            return ZERO_CMD

        speed = min(self.rate, max(self.min_rate, 1.5 * remaining))
        return (0.0, 0.0, 0.0, self.sign * speed)


class Forward(object):
    """沿起始机头方向前进指定距离，按刹车曲线自动收油."""

    name = "FORWARD"

    def __init__(self, distance=3.0, speed=1.5, accel=2.0, tolerance=0.02):
        self.distance = float(distance)
        self.speed = float(speed)
        self.accel = float(accel)
        self.tolerance = float(tolerance)
        self._start = None
        self._dir = None
        self.done = False

    def update(self, pose, dt=0.0):
        if self.done:
            return ZERO_CMD
        if self._start is None:
            self._start = (pose.x, pose.y)
            self._dir = (math.cos(pose.yaw), math.sin(pose.yaw))

        travelled = ((pose.x - self._start[0]) * self._dir[0] +
                     (pose.y - self._start[1]) * self._dir[1])
        remaining = self.distance - travelled
        if remaining <= self.tolerance:
            self.done = True
            return ZERO_CMD
        speed = min(self.speed, math.sqrt(2.0 * self.accel * remaining))
        return (speed, 0.0, 0.0, 0.0)


class MacroRunner(object):
    """按键 -> 宏对象的状态机；同一时刻只允许一个宏在跑."""

    def __init__(self, takeoff_altitude=1.5, spin_degrees=360.0,
                 forward_distance=3.0, forward_speed=1.5, brake_accel=2.0,
                 alt_kp=2.0, max_vert_speed=1.0, spin_rate=1.0,
                 altitude_tolerance=0.05, settle_ticks=6):
        self.takeoff_altitude = float(takeoff_altitude)
        self.spin_degrees = float(spin_degrees)
        self.forward_distance = float(forward_distance)
        self.forward_speed = float(forward_speed)
        self.brake_accel = float(brake_accel)
        self.alt_kp = float(alt_kp)
        self.max_vert_speed = float(max_vert_speed)
        self.spin_rate = float(spin_rate)
        self.altitude_tolerance = float(altitude_tolerance)
        self.settle_ticks = int(settle_ticks)
        self._macro = None

    # ------------------------------------------------------------------ 状态
    @property
    def active(self):
        return self._macro is not None

    @property
    def name(self):
        return self._macro.name if self._macro else ""

    def trigger(self, key, pose):
        """按键触发宏；未知按键或没有位姿返回 False，不改变当前状态."""
        key = str(key)
        if key not in MACRO_KEYS or pose is None:
            return False
        if key == "1":
            self._macro = Takeoff(self.takeoff_altitude, self.alt_kp,
                                  self.max_vert_speed, self.altitude_tolerance,
                                  self.settle_ticks)
        elif key == "2":
            self._macro = Spin(self.spin_degrees, self.spin_rate)
        else:
            self._macro = Forward(self.forward_distance, self.forward_speed,
                                  self.brake_accel)
        return True

    def cancel(self):
        self._macro = None

    def update(self, pose, dt=0.0):
        """推进一帧，返回机体系速度；宏完成或没有宏时返回全零."""
        if self._macro is None:
            return ZERO_CMD
        if pose is None:
            self.cancel()
            return ZERO_CMD
        cmd = self._macro.update(pose, dt)
        if self._macro.done:
            self._macro = None
        return cmd
