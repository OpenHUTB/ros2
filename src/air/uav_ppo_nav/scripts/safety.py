#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全速度包络：把策略输出的世界系速度约束在可执行的安全范围内.

策略是在轻量质点环境里训练的，部署到 CarlaAir/AirSim 或真机时还需要一层
**与策略解耦**的兜底，避免"策略输出很大、但前方就是墙"这类失败：

* 速度限幅：水平/垂直速度夹到与动作映射一致的上限；
* 近障减速：沿速度方向的扇形净空越小，水平速度按比例衰减；
* 急停：净空低于 ``stop_distance`` 时输出零水平速度（垂直速度保留，便于脱离）。

本模块只依赖 numpy，与 ROS 解耦，可离线单测。
"""
from __future__ import annotations

import math

import numpy as np

from policy import MAX_HORIZ_SPEED, MAX_RANGE, MAX_VERT_SPEED


class SafetyEnvelope(object):
    """按前方净空缩放世界系速度的安全层。"""

    def __init__(self, stop_distance=0.8, slow_distance=3.0,
                 max_horiz_speed=MAX_HORIZ_SPEED, max_vert_speed=MAX_VERT_SPEED,
                 max_range=MAX_RANGE, sector_window=1):
        if not stop_distance >= 0.0:
            raise ValueError('stop_distance must be non-negative')
        if not stop_distance < slow_distance:
            raise ValueError('slow_distance must be greater than stop_distance')
        if max_horiz_speed <= 0 or max_vert_speed <= 0 or max_range <= 0:
            raise ValueError('speed and range limits must be positive')
        self.stop_distance = float(stop_distance)
        self.slow_distance = float(slow_distance)
        self.max_horiz_speed = float(max_horiz_speed)
        self.max_vert_speed = float(max_vert_speed)
        self.max_range = float(max_range)
        self.sector_window = max(0, int(sector_window))

    # ------------------------------------------------------------ 净空
    def sector_of(self, vx, vy, n_sectors):
        """世界系速度方向所在的水平扇形下标（与观测直方图分扇方式一致）。"""
        width = 2.0 * math.pi / max(int(n_sectors), 1)
        angle = math.atan2(vy, vx)
        return int(math.floor((angle + width / 2.0) / width)) % int(n_sectors)

    def clearance_m(self, hist, sector):
        """取该扇形及相邻扇形的最近障碍距离（米）；hist 为 0..1 归一化值。"""
        hist = np.asarray(hist, dtype=np.float64).reshape(-1)
        if hist.size == 0:
            return self.max_range
        index = int(sector) % hist.size
        indices = [(index + offset) % hist.size
                   for offset in range(-self.sector_window, self.sector_window + 1)]
        nearest = float(np.min(hist[indices]))
        return max(0.0, min(nearest, 1.0)) * self.max_range

    def min_clearance_m(self, hist):
        """所有扇形里的最近障碍距离（米），用于状态/日志。"""
        hist = np.asarray(hist, dtype=np.float64).reshape(-1)
        if hist.size == 0:
            return self.max_range
        return max(0.0, min(float(np.min(hist)), 1.0)) * self.max_range

    # ------------------------------------------------------------ 过滤
    def filter(self, vx, vy, vz, hist):
        """限制速度并返回 ``(vx, vy, vz, info)``。

        info 字段：``clearance_m`` / ``scale`` / ``braking`` / ``sector``。
        悬停（水平速度接近 0）时不判定方向，也不会触发急停。
        """
        vx = float(np.clip(vx, -self.max_horiz_speed, self.max_horiz_speed))
        vy = float(np.clip(vy, -self.max_horiz_speed, self.max_horiz_speed))
        vz = float(np.clip(vz, -self.max_vert_speed, self.max_vert_speed))

        hist = np.asarray(hist, dtype=np.float64).reshape(-1)
        info = {'clearance_m': self.max_range, 'scale': 1.0,
                'braking': False, 'sector': None}
        if hist.size == 0 or math.hypot(vx, vy) < 1e-6:
            return vx, vy, vz, info

        sector = self.sector_of(vx, vy, hist.size)
        clearance = self.clearance_m(hist, sector)
        info['sector'] = sector
        info['clearance_m'] = round(clearance, 3)

        if clearance <= self.stop_distance:
            info['scale'] = 0.0
            info['braking'] = True
            return 0.0, 0.0, vz, info
        if clearance < self.slow_distance:
            scale = (clearance - self.stop_distance) / (self.slow_distance - self.stop_distance)
            info['scale'] = round(scale, 3)
            return vx * scale, vy * scale, vz, info
        return vx, vy, vz, info
