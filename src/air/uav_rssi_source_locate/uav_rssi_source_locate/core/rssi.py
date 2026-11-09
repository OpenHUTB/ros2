#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RSSI path-loss model and distance inversion.

Single-line summary: log-distance path loss rssi(d) = rssi0 − 10·n·log10(d/d0)
with a receiver floor; inversion recovers distance from one RSSI sample,
with quantization error propagating to a few percent of range.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RssiCfg:
    """电波模型参数（对数距离路损）。"""

    rssi0: float = -40.0     # dBm，d0 = 1 m 处的参考接收强度
    n: float = 2.2           # 路径损耗指数（自由空间 2，城区 2.7~3.5）
    d0: float = 1.0          # m，参考距离
    floor: float = -76.0     # dBm，接收机灵敏度底噪（对应约 43 m 可检测半径）
    quant_db: float = 0.5    # dB，RSSI 量化步长（实测芯片常见 0.5~1 dB）

    def __post_init__(self):
        if self.n <= 0.0 or self.d0 <= 0.0:
            raise ValueError("n / d0 must be positive")
        if self.quant_db < 0.0:
            raise ValueError("quant_db must be non-negative")

    def rssi_at(self, dist_m: float) -> float:
        """真实距离 → 量化后的 RSSI 读数（低于底噪返回 -inf 语义：None）。"""
        if dist_m <= 0.0:
            raise ValueError("distance must be positive")
        raw = self.rssi0 - 10.0 * self.n * np.log10(dist_m / self.d0)
        if raw < self.floor:
            return float("nan")          # 收不到：调用方按未检测处理
        q = self.quant_db
        # 就近取整而不是向下取整：floor 会让读数系统性偏低 ~q/2，
        # 反演距离整体偏大 ~2%，最小二乘吸收不掉这种固定偏差
        return float(np.round(raw / q) * q) if q > 0.0 else float(raw)

    def distance_from(self, rssi_dbm: float) -> float:
        """一次 RSSI 读数 → 距离反演（dBm）。

        量化 0.5 dB 在 40 m 处对应约 ±1.3 m（3%）的距离误差——这就是
        单点测距的物理精度上限，定位误差靠多点最小二乘压下去。
        """
        if not np.isfinite(rssi_dbm):
            raise ValueError("no signal (below floor)")
        d = self.d0 * 10.0 ** ((self.rssi0 - rssi_dbm) / (10.0 * self.n))
        return float(min(max(d, self.d0), 10.0 ** 6))

    @property
    def detect_range_m(self) -> float:
        """底噪对应的可检测半径。"""
        return self.d0 * 10.0 ** ((self.rssi0 - self.floor) / (10.0 * self.n))
