"""差速驱动移动机器人运动学模型。

仅依赖标准库与 numpy，可在无 ROS 环境下独立运行，便于离线仿真与单元测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Pose:
    """机器人二维位姿。"""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)


@dataclass
class DiffDriveRobot:
    """两轮差速驱动机器人。

    状态方程（离散化）：
        x_{k+1} = x_k + v * cos(theta_k) * dt
        y_{k+1} = y_k + v * sin(theta_k) * dt
        theta_{k+1} = theta_k + omega * dt
    """

    wheel_base: float = 0.50      # 轮距 (m)
    wheel_radius: float = 0.10    # 轮半径 (m)
    max_v: float = 1.2            # 最大线速度 (m/s)
    max_w: float = 1.8            # 最大角速度 (rad/s)
    max_accel: float = 1.5        # 最大线加速度 (m/s^2)
    dt: float = 0.05              # 控制周期 (s)
    pose: Pose = field(default_factory=Pose)
    v: float = 0.0                # 当前线速度
    w: float = 0.0                # 当前角速度

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """复位到指定位姿并清空速度。"""
        self.pose = Pose(x=x, y=y, theta=theta)
        self.v = 0.0
        self.w = 0.0

    def _limit(self, v_cmd: float, w_cmd: float) -> tuple[float, float]:
        """对速度指令做加速度与幅值限幅。"""
        # 加速度限幅
        dv = v_cmd - self.v
        dv_max = self.max_accel * self.dt
        if abs(dv) > dv_max:
            v_cmd = self.v + math.copysign(dv_max, dv)
        # 幅值限幅
        v_cmd = float(np.clip(v_cmd, -self.max_v, self.max_v))
        w_cmd = float(np.clip(w_cmd, -self.max_w, self.max_w))
        return v_cmd, w_cmd

    def step(self, v_cmd: float, w_cmd: float) -> Pose:
        """执行一个控制周期，返回更新后的位姿。"""
        v, w = self._limit(v_cmd, w_cmd)
        self.v, self.w = v, w
        self.pose.x += v * math.cos(self.pose.theta) * self.dt
        self.pose.y += v * math.sin(self.pose.theta) * self.dt
        self.pose.theta += w * self.dt
        # 角度归一化到 [-pi, pi]
        self.pose.theta = math.atan2(math.sin(self.pose.theta), math.cos(self.pose.theta))
        return self.pose

    def wheel_speeds(self) -> tuple[float, float]:
        """由 (v, w) 反解左右轮转速 (rad/s)，用于底层电机指令。"""
        vl = self.v - self.w * self.wheel_base / 2.0
        vr = self.v + self.w * self.wheel_base / 2.0
        return vl / self.wheel_radius, vr / self.wheel_radius

    def to_robot_frame(self, x: float, y: float) -> tuple[float, float]:
        """把世界坐标点转换到车体坐标系（前为 x，左为 y）。"""
        c, s = math.cos(self.pose.theta), math.sin(self.pose.theta)
        dx, dy = x - self.pose.x, y - self.pose.y
        return dx * c + dy * s, -dx * s + dy * c
