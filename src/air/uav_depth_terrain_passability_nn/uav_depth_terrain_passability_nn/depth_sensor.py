#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机载深度地形传感器：分左中右三扇区采样地形台阶与坡度。

传感器在机体前方扇形范围内采样地面点，统计每个扇区的平均高度、
最大台阶高度与平均坡度，作为通行度网络的输入特征。模块不依赖 ROS。
"""

from __future__ import annotations

import numpy as np

from . import terrain

SECTORS = ("left", "center", "right")
FORWARD_DIST = (1.5, 2.5, 3.5)
LATERAL_BAND = {
    "left": (0.8, 1.5),
    "center": (-0.4, 0.4),
    "right": (-1.5, -0.8),
}


class DepthTerrainSensor:
    """三扇区地形深度传感器。"""

    def observe(self, state) -> dict:
        x = float(state.x if hasattr(state, "x") else state["x"])
        y = float(state.y if hasattr(state, "y") else state["y"])
        yaw = float(state.yaw if hasattr(state, "yaw") else state["yaw"])
        cy, sy = math_cos(yaw), math_sin(yaw)

        features: list[float] = []
        sector_points: dict[str, list[tuple[float, float]]] = {s: [] for s in SECTORS}

        for sector in SECTORS:
            lat0, lat1 = LATERAL_BAND[sector]
            heights, steps, slopes = [], [], []
            for fwd in FORWARD_DIST:
                for lat in np.linspace(lat0, lat1, 3):
                    wx = x + cy * fwd - sy * lat
                    wy = y + sy * fwd + cy * lat
                    heights.append(terrain.height(wx, wy))
                    steps.append(terrain.step_height(wx, wy))
                    slopes.append(terrain.slope(wx, wy))
                    sector_points[sector].append((wx, wy))
            features.extend([
                float(np.mean(heights)),
                float(np.mean(steps)),
                float(np.mean(slopes)),
            ])

        return {
            "features": np.array(features, dtype=float),
            "sector_points": sector_points,
            "on_block": terrain.is_blocked(x, y),
        }

    @staticmethod
    def normalize(features: np.ndarray) -> np.ndarray:
        """把台阶/坡度特征归一化到网络输入区间。"""
        f = np.asarray(features, dtype=float).reshape(3, 3)
        f[:, 0] = np.clip(f[:, 0] / terrain.BLOCK_HEIGHT, -1.0, 1.0)
        f[:, 1] = np.clip(f[:, 1] / terrain.STEP_LIMIT, -1.0, 1.0)
        f[:, 2] = np.clip(f[:, 2] / terrain.SLOPE_LIMIT, -1.0, 1.0)
        return f.reshape(-1)


def math_cos(a: float) -> float:
    return float(np.cos(a))


def math_sin(a: float) -> float:
    return float(np.sin(a))
