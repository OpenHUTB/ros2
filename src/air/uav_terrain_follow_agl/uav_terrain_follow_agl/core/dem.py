#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数字高程模型（DEM）与其误差模型。

真实低空作业用的地形跟随不靠「看见地形」，而是靠**先验高程库 + GNSS 定位**：
机上存一份 DEM，用当前位置查出前方几秒的高程，提前把爬升做掉。DEM 的误差是
系统性的（垂直偏置、水平移位、网格分辨率），不会像雷达噪声那样被平均掉，所以
必须由雷达高度表的慢速积分环来校正——这正是本模块 C 策略做的两件事。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np

from .terrain import TerrainField


@dataclass(frozen=True)
class DemConfig:
    """DEM 误差模型（三级：理想 / 实用 / 劣化）。"""

    grid_m: float = 2.0        # 网格间隔，0 表示连续理想高程
    bias_m: float = 0.8        # 垂直偏置（整片一致，积分环能吃掉）
    shift_x_m: float = 3.0     # 沿 x 的水平移位（地形特征被提前/滞后）
    shift_y_m: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.grid_m) or self.grid_m < 0.0:
            raise ValueError(f"DEM 网格间隔必须非负，收到 {self.grid_m!r}")
        for key, val in (("bias_m", self.bias_m),
                         ("shift_x_m", self.shift_x_m),
                         ("shift_y_m", self.shift_y_m)):
            if not math.isfinite(val):
                raise ValueError(f"DEM 参数 {key} 必须是有限数，收到 {val!r}")

    def as_dict(self) -> Dict[str, float]:
        return {"grid_m": float(self.grid_m), "bias_m": float(self.bias_m),
                "shift_x_m": float(self.shift_x_m), "shift_y_m": float(self.shift_y_m)}


IDEAL_DEM = DemConfig(grid_m=0.0, bias_m=0.0, shift_x_m=0.0)
"""理想 DEM：抽查即真值，用来把「控制律的作用」和「DEM 误差的作用」分开。"""

PRACTICAL_DEM = DemConfig(grid_m=2.0, bias_m=0.8, shift_x_m=3.0)
"""实用 DEM：2 m 网格、0.8 m 偏置、3 m 前移（10 m/s 时相当于 0.3 s）。"""

DEGRADED_DEM = DemConfig(grid_m=8.0, bias_m=4.0, shift_x_m=12.0)
"""劣化 DEM：8 m 网格、4 m 偏置、12 m 前移，模拟开源高程库叠加坐标误差。"""


class DigitalElevationModel:
    """按网格预采样的 DEM，带垂直偏置与水平移位。"""

    def __init__(self, terrain: TerrainField, config: DemConfig = PRACTICAL_DEM,
                 x_range=(0.0, 720.0), y_range=(-80.0, 80.0)):
        self.terrain = terrain
        self.config = config
        self.x_range = (float(x_range[0]), float(x_range[1]))
        self.y_range = (float(y_range[0]), float(y_range[1]))
        if self.config.grid_m <= 0.0:
            self._grid = None
            return
        g = float(self.config.grid_m)
        self._grid = g
        self._xs = np.arange(self.x_range[0], self.x_range[1] + g * 0.5, g)
        self._ys = np.arange(self.y_range[0], self.y_range[1] + g * 0.5, g)
        # 网格值取真值；移位在查询时施加（相当于高程库整体平移）
        self._zs = np.asarray(
            terrain.elevation(self._xs[None, :], self._ys[:, None]), dtype=float
        )

    # ---------------------------------------------------------------- 查询
    def raw(self, x, y=0.0):
        """DEM 读数（不含偏置，只做移位与网格化）。"""
        xq = np.asarray(x, dtype=float) - self.config.shift_x_m
        yq = np.asarray(y, dtype=float) - self.config.shift_y_m
        if self._grid is None:
            return np.asarray(self.terrain.elevation(xq, yq), dtype=float)
        return self._bilinear(xq, yq)

    def height(self, x, y=0.0):
        """带垂直偏置的 DEM 高度（米）。"""
        h = self.raw(x, y) + self.config.bias_m
        return float(h) if np.ndim(h) == 0 else h

    def error(self, x, y=0.0):
        """DEM 相对真值的误差（米），正表示 DEM 偏高。"""
        return np.asarray(self.height(x, y), dtype=float) - np.asarray(
            self.terrain.elevation(x, y), dtype=float)

    def _bilinear(self, x, y):
        g = self._grid
        nx = self._xs.size
        ny = self._ys.size
        ix = np.clip((x - self._xs[0]) / g, 0.0, nx - 1.0 - 1e-9)
        iy = np.clip((y - self._ys[0]) / g, 0.0, ny - 1.0 - 1e-9)
        i0 = np.floor(ix).astype(int)
        j0 = np.floor(iy).astype(int)
        fx = ix - i0
        fy = iy - j0
        z00 = self._zs[j0, i0]
        z01 = self._zs[j0, np.minimum(i0 + 1, nx - 1)]
        z10 = self._zs[np.minimum(j0 + 1, ny - 1), i0]
        z11 = self._zs[np.minimum(j0 + 1, ny - 1), np.minimum(i0 + 1, nx - 1)]
        return (z00 * (1 - fx) * (1 - fy) + z01 * fx * (1 - fy)
                + z10 * (1 - fx) * fy + z11 * fx * fy)

    # ---------------------------------------------------------------- 统计
    def stats(self, x0: float, x1: float, y: float = 0.0, n: int = 601) -> Dict[str, float]:
        """一段航迹上的 DEM 误差统计，用于解释间隙余量的来源。"""
        xs = np.linspace(float(x0), float(x1), int(n))
        err = self.error(xs, y)
        return {
            "mean_error_m": float(np.mean(err)),
            "rms_error_m": float(np.sqrt(np.mean(err ** 2))),
            "max_error_m": float(np.max(np.abs(err))),
        }
