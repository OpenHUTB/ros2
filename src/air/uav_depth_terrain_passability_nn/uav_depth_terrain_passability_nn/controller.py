#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地形通行决策运动控制器：网络决策 -> 航向与速度更新。

根据最优扇区设定目标横向偏移，航向 PD 转向该扇区，
前飞速度由速度指令与所选扇区通行度共同标定。控制器不依赖 ROS。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SECTOR_LAT = (1.4, 0.0, -1.4)   # 左、中、右扇区目标横向偏移 m


@dataclass
class TerrainState:
    """平面运动状态（高度保持在悬停高度）。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    altitude: float = 1.0


@dataclass
class ControllerParams:
    v_max: float = 1.00
    kp_yaw: float = 2.4
    vel_damping: float = 3.0
    min_forward: float = 0.18
    sector_hyst: float = 0.12      # 扇区切换滞回裕度


class TerrainController:
    """把通行度决策积分成下一时刻状态。"""

    def __init__(self, params: ControllerParams | None = None):
        self.p = params or ControllerParams()
        self.last_sector: int | None = None

    def _choose_sector(self, passability: np.ndarray) -> int:
        best = int(np.argmax(passability))
        if self.last_sector is not None:
            if passability[self.last_sector] + self.p.sector_hyst >= passability[best]:
                return self.last_sector
        self.last_sector = best
        return best

    def step(self, state: TerrainState, decision: dict, dt: float) -> TerrainState:
        p = self.p
        sector = self._choose_sector(np.asarray(decision["passability"], dtype=float))
        passability = float(decision["passability"][sector])
        speed_cmd = float(np.clip(decision["speed"], -1.0, 1.0))

        lat_target = SECTOR_LAT[sector]
        speed_cmd = max(0.0, speed_cmd)
        v_forward = max(p.min_forward, speed_cmd * p.v_max) * (0.7 + 0.3 * passability)
        yaw_des = float(np.arctan2(lat_target, 1.0))
        yaw_rate = p.kp_yaw * self._wrap(yaw_des - state.yaw)

        des_vx = np.cos(state.yaw) * v_forward
        des_vy = np.sin(state.yaw) * v_forward
        alpha = min(1.0, p.vel_damping * dt)
        vx = state.vx + (des_vx - state.vx) * alpha
        vy = state.vy + (des_vy - state.vy) * alpha

        return TerrainState(
            x=state.x + vx * dt,
            y=state.y + vy * dt,
            yaw=self._wrap(state.yaw + yaw_rate * dt),
            vx=vx, vy=vy, altitude=state.altitude,
        )

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
