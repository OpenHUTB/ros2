#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达高度表模型：采样保持 + 传输延迟 + 量化 + 噪声 + 丢帧。

低空定高飞行的主传感器是无线电/雷达高度表，它测的是**对地**距离，正好是地形
跟随要控的量。但它有两个躲不掉的性质：

* 延迟：20 Hz 采样、链路与滤波合计 100~200 ms，在 10 m/s 速度下就是 1~2 m 的
  空间滞后——控制器永远在跟「刚才的地形」；
* 噪声与偏置：σ 约 0.2~0.3 m，另有一个整架次不变的安装/标定偏置。

本类逐帧复现这些性质，供离线与闭环仿真共用；不依赖 ROS，方便单测。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass(frozen=True)
class AltimeterConfig:
    rate_hz: float = 20.0
    latency_s: float = 0.12
    sigma_m: float = 0.25
    bias_m: float = 0.15
    quant_m: float = 0.05
    dropout_p: float = 0.01

    def __post_init__(self) -> None:
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError(f"采样率必须为正，收到 {self.rate_hz!r}")
        if not math.isfinite(self.latency_s) or self.latency_s < 0.0:
            raise ValueError(f"延迟必须非负，收到 {self.latency_s!r}")
        if not math.isfinite(self.sigma_m) or self.sigma_m < 0.0:
            raise ValueError(f"噪声标准差必须非负，收到 {self.sigma_m!r}")
        if not math.isfinite(self.bias_m) or self.bias_m < 0.0:
            raise ValueError(f"偏置幅度必须非负，收到 {self.bias_m!r}")
        if not math.isfinite(self.quant_m) or self.quant_m < 0.0:
            raise ValueError(f"量化步长必须非负，收到 {self.quant_m!r}")
        if not 0.0 <= self.dropout_p <= 1.0:
            raise ValueError(f"丢帧概率必须在 [0,1]，收到 {self.dropout_p!r}")

    @property
    def period_s(self) -> float:
        return 1.0 / self.rate_hz

    def as_dict(self) -> dict:
        return {
            "rate_hz": float(self.rate_hz),
            "latency_s": float(self.latency_s),
            "sigma_m": float(self.sigma_m),
            "bias_m": float(self.bias_m),
            "quant_m": float(self.quant_m),
            "dropout_p": float(self.dropout_p),
        }


class RadarAltimeter:
    """采样→量化→（按延迟出队）的状态机。

    ``update(t, agl_true)`` 在每个控制周期调用：到采样时刻就压一帧进延迟队列，
    同时把「已经过期」的帧推出作为当前读数。``reading`` 是采样保持后的值，
    与真实传感器输出一致——两次采样之间读数不变。
    """

    def __init__(self, config: AltimeterConfig = AltimeterConfig(), seed: int = 0):
        import numpy as np

        self.config = config
        self._np = np
        self._rng = np.random.default_rng(int(seed))
        # 安装/标定偏置：整架次不变，所以只能靠 DEM 慢环或标定消掉
        self.bias = float(self._rng.normal(0.0, config.bias_m)) if config.bias_m > 0 else 0.0
        self.reset()

    # ---------------------------------------------------------------- 状态
    def reset(self) -> None:
        self._next_sample = 0.0
        self._queue: Deque[Tuple[float, float]] = deque()
        self._reading: Optional[float] = None
        self._reading_t: Optional[float] = None
        self.n_samples = 0
        self.n_dropout = 0

    @property
    def reading(self) -> Optional[float]:
        """当前保持的测量值（米）；还没出过数就是 None。"""
        return self._reading

    @property
    def reading_time(self) -> Optional[float]:
        return self._reading_t

    def age(self, t: float) -> float:
        """当前读数的「年龄」（秒），没有读数时返回 inf。"""
        if self._reading_t is None:
            return float("inf")
        return float(t) - float(self._reading_t)

    # ---------------------------------------------------------------- 采样
    def measure(self, agl_true: float) -> float:
        """给一帧真实离地高度，返回该帧的测量值（未加延迟）。"""
        cfg = self.config
        val = float(agl_true) + self.bias + float(self._rng.normal(0.0, cfg.sigma_m))
        if cfg.quant_m > 0.0:
            val = round(val / cfg.quant_m) * cfg.quant_m
        return float(val)

    def update(self, t: float, agl_true: float) -> bool:
        """推进到时刻 ``t``；返回本周期是否产生了新样本。"""
        produced = False
        # 用 while 兼容「控制周期比传感器周期长」的调用方式
        while t + 1e-12 >= self._next_sample:
            if self._rng.random() < self.config.dropout_p:
                self.n_dropout += 1          # 丢帧：不采，队列里保持上一帧
            else:
                self._queue.append((float(t), self.measure(agl_true)))
                self.n_samples += 1
            self._next_sample += self.config.period_s
            produced = True
        # 出队：只放行「采样时刻 + 延迟 ≤ 现在」的帧
        while self._queue and self._queue[0][0] + self.config.latency_s <= t + 1e-12:
            ts, val = self._queue.popleft()
            self._reading, self._reading_t = val, ts
        return produced


def latency_error_m(speed_mps: float, slope: float, latency_s: float) -> float:
    """延迟在恒坡上造成的空间滞后：Δ = v·tanθ·τ（米）。

    这是解析式，用来和闭环实测的稳态 AGL 误差对照。
    """
    return abs(float(speed_mps)) * abs(float(slope)) * abs(float(latency_s))
