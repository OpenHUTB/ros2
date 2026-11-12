#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""循迹运动控制器：神经网络指令 -> 速度与偏航更新。

内环把策略输出的前飞指令与偏航指令转换为机体运动，
并叠加切线航向 PD 保持，保证沿标线稳定跟踪。控制器不依赖 ROS。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import track


@dataclass
class TrackState:
    """平面跟踪状态（高度保持在悬停高度）。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    altitude: float = 1.0


@dataclass
class ControllerParams:
    v_max: float = 1.00          # 最大前飞速度 m/s
    yaw_max: float = 1.2          # 最大偏航角速度 rad/s
    kp_tangent: float = 0.6       # 切线航向 PD 增益
    vel_damping: float = 1.8      # 速度跟踪一阶阻尼
    min_forward: float = 0.12     # 最低前飞速度 m/s


class TrackController:
    """把策略指令积分成下一时刻状态。"""

    def __init__(self, params: ControllerParams | None = None):
        self.p = params or ControllerParams()

    def step(self, state: TrackState, policy_out: np.ndarray, dt: float) -> TrackState:
        p = self.p
        v_cmd = float(np.clip(policy_out[0], -1.0, 1.0))
        yaw_cmd = float(np.clip(policy_out[1], -1.0, 1.0))

        v_forward = max(p.min_forward, v_cmd * p.v_max)
        tangent = track.reference_tangent(state.x)
        yaw_rate = p.kp_tangent * self._wrap(tangent - state.yaw) \
            + yaw_cmd * p.yaw_max * 0.5

        des_vx = np.cos(state.yaw) * v_forward
        des_vy = np.sin(state.yaw) * v_forward
        alpha = min(1.0, p.vel_damping * dt)
        vx = state.vx + (des_vx - state.vx) * alpha
        vy = state.vy + (des_vy - state.vy) * alpha

        new = TrackState(
            x=state.x + vx * dt,
            y=state.y + vy * dt,
            yaw=self._wrap(state.yaw + yaw_rate * dt),
            vx=vx,
            vy=vy,
            altitude=state.altitude,
        )
        return new

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
