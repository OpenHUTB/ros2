#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双侧墙通道居中 PD 巡航控制核心（纯 Python，不依赖 rclpy / 物理仿真器）。

场景：两轮差速底盘在一条笔直通道内前进，激光分别测出左侧墙距离 ``d_left``、
右侧墙距离 ``d_right`` 和正前方距离 ``d_forward``。控制器只用这三路测距，
把车体保持在通道中线上：

- 横向偏差 ``e = (d_left - d_right) / 2``，居中时 ``e = 0``；
- PD 输出角速度 ``omega = Kp * e + Kd * de/dt``，偏差越大、纠偏越急；
- 前进速度随角速度衰减，转角越大车速越低；
- 正前方距离小于 ``forward_safety`` 时进入 ``BLOCKED`` 状态并零速停车，
  等前方距离恢复到 ``forward_recover`` 后再继续巡航。

几何与控制都不读取仿真真值，便于离线单元测试，也便于在没有
Gazebo / CARLA / MuJoCo 的环境里跑通闭环。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class CenteringParams:
    """控制器可调参数（与 config/params.yaml 对应）。"""

    kp: float = 1.2
    kd: float = 0.6
    v_max: float = 0.4
    v_floor: float = 0.08
    w_max: float = 1.2
    forward_safety: float = 0.6
    forward_recover: float = 0.9
    scan_timeout: float = 0.5


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CorridorCenteringController:
    """根据左右墙与前方测距，输出差速底盘的线速度 / 角速度指令。"""

    def __init__(self, params: CenteringParams | None = None):
        self.params = params or CenteringParams()
        self.state = "CENTER"
        self._prev_error: float | None = None
        self._prev_time: float | None = None
        self._last_valid_time: float | None = None

    def reset(self) -> None:
        """清空积分 / 微分历史，状态复位到巡航。"""
        self.state = "CENTER"
        self._prev_error = None
        self._prev_time = None
        self._last_valid_time = None

    def step(self, d_left: float, d_right: float, d_forward: float,
             now: float) -> dict:
        """执行一步控制。

        参数：
          d_left:  左侧墙距离（m）
          d_right: 右侧墙距离（m）
          d_forward: 正前方障碍距离（m）
          now: 当前时间（s，单调递增）

        返回 dict：
          v, w:     线速度（m/s）、角速度（rad/s）
          e:        横向偏差（m）
          de:       偏差变化率（m/s）
          state:    ``CENTER`` / ``BLOCKED``
          d_left, d_right, d_forward: 回传本次测距
        """
        p = self.params

        # 测距超时保护：超过 scan_timeout 没收到新测距时按前方受阻处理。
        # 超时只作为临时保护：这里同步刷新时间戳，断流恢复后下一帧即可退出
        # BLOCKED 回到巡航，不会被上一次超时时间一直锁死。
        if self._last_valid_time is not None and now - self._last_valid_time > p.scan_timeout:
            self.state = "BLOCKED"
            self._last_valid_time = now
            return self._result(0.0, 0.0, 0.0, 0.0, d_left, d_right, d_forward)
        self._last_valid_time = now

        error = (d_left - d_right) / 2.0

        dt = None
        if self._prev_time is not None and now > self._prev_time:
            dt = now - self._prev_time
        derivative = 0.0
        if dt is not None and self._prev_error is not None:
            derivative = (error - self._prev_error) / dt

        # 状态机：CENTER -> BLOCKED（前方过近）-> CENTER（前方恢复）。
        if self.state == "CENTER" and d_forward < p.forward_safety:
            self.state = "BLOCKED"
        elif self.state == "BLOCKED" and d_forward >= p.forward_recover:
            self.state = "CENTER"

        if self.state == "BLOCKED":
            v, w = 0.0, 0.0
        else:
            w = _clip(p.kp * error + p.kd * derivative, -p.w_max, p.w_max)
            v = p.v_max * max(p.v_floor, 1.0 - abs(w) / p.w_max)

        self._prev_error = error
        self._prev_time = now
        return self._result(v, w, error, derivative, d_left, d_right, d_forward)

    def _result(self, v: float, w: float, error: float, derivative: float,
                d_left: float, d_right: float, d_forward: float) -> dict:
        return {
            "v": v,
            "w": w,
            "e": error,
            "de": derivative,
            "state": self.state,
            "d_left": d_left,
            "d_right": d_right,
            "d_forward": d_forward,
        }


def diff_drive_step(pose: tuple[float, float, float], v: float, w: float,
                    dt: float) -> tuple[float, float, float]:
    """差速底盘 2D 运动学积分一步（供离线仿真与节点内虚拟测距复用）。

    pose = (x, y, yaw)，返回积分后的新位姿。
    """
    x, y, yaw = pose
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += w * dt
    return x, y, yaw
