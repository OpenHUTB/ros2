#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""递推最小二乘（带遗忘因子），用于在线自适应最后一层。

只更新最后一层而不是全网：在线数据是**时序相关**的（一条轨迹上的速度不可能
跳变），对整个网络做梯度更新会沿相关方向把前面几层冲垮；而最后一层是线性
回归，RLS 对相关数据的收敛性有闭式保证，且一次迭代只要 O(h²)（h=65，
实测 6 µs/步，100 Hz 下占用 < 0.1% CPU）。
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

__all__ = ["RecursiveLeastSquares"]


class RecursiveLeastSquares:
    """线性模型 ``Y = ΦᵀΘ`` 的在线辨识，支持多输出（Θ 为 n×m 矩阵）。

    两个超参决定"在线修正有多大胆"：

    ``delta``
        θ 初值的协方差（把离线解当成一个观测）。这是**最关键的一个**：
        取 1e3 相当于认为离线解完全不可信，第一次更新就按 k≈φ/φᵀφ 把 θ
        精确拟合到单个样本上，定点悬停场景实测 RMSE 0.14~0.25 m
        （比不用 RLS 还差）；取 1e2 后同一组工况降到 0.019~0.027 m。

    ``trace_growth``
        协方差上限。回归向量长期只占一个方向时（定点悬停），P 会在零空间里
        按 1/λ 指数增长，等阵风把 φ 转到新方向时增益突然变大。超限时**按比例
        缩放**协方差而不是重置成初值——重置等价于声称"θ̂ 一无所知"，下一次
        更新照样跳，没有意义。
    """

    def __init__(self, n_params: int, n_out: int = 1, lam: float = 0.998,
                 delta: float = 1.0e2, min_reg: float = 0.05,
                 trace_growth: float = 4.0, leak: float = 0.0,
                 theta_clip: float = 30.0):
        self.n = int(n_params)
        self.m = int(n_out)
        self.lam = float(lam)
        self.delta = float(delta)
        self.min_reg = float(min_reg)
        self.trace_growth = float(trace_growth)
        self.leak = float(leak)
        self.theta_clip = float(theta_clip)
        self.reset()

    def reset(self) -> None:
        anchor = getattr(self, "theta0", None)
        self.theta = anchor.copy() if anchor is not None \
            else np.zeros((self.n, self.m))
        self.P = np.eye(self.n) * self.delta
        self.trace_cap = self.trace_growth * self.n * self.delta
        self.updates = 0
        self.skipped = 0
        self.resets = 0

    # ------------------------------------------------------------ 更新
    @staticmethod
    def _energy(phi: np.ndarray) -> float:
        return float(np.mean(np.asarray(phi, dtype=float) ** 2))

    def update(self, phi: np.ndarray, y: np.ndarray) -> bool:
        """喂入一组 (φ, y)。返回是否真的更新了。"""
        phi = np.asarray(phi, dtype=float).reshape(-1)
        y = np.asarray(y, dtype=float).reshape(-1)
        if self._energy(phi) < self.min_reg * self.min_reg:
            self.skipped += 1
            return False
        tr = float(np.trace(self.P))
        if tr > self.trace_cap:
            self.P *= self.trace_cap / tr
            self.resets += 1

        Pphi = self.P @ phi
        denom = self.lam + float(phi @ Pphi)
        if denom < 1e-9:
            self.skipped += 1
            return False
        k = Pphi / denom
        err = y - self.theta.T @ phi
        self.theta = self.theta + np.outer(k, err)
        # 可选泄漏项：把 θ 往先验（离线解）拉，用于需要长时间在线、
        # 担心参数缓慢漂移的场合。默认关闭——真正解决零空间漂移的是
        # delta（先验协方差），泄漏只是软约束，挡不住单步大跳。
        if self.leak > 0.0:
            self.theta -= self.leak * (self.theta - self.theta0)
        if self.theta_clip > 0.0:
            np.clip(self.theta, -self.theta_clip, self.theta_clip, out=self.theta)
        # 协方差更新，同步做对称化：λ→1 时 (P − kφᵀP)/λ 会缓慢丢失对称性
        self.P = (self.P - np.outer(k, Pphi)) / self.lam
        self.P = 0.5 * (self.P + self.P.T)
        self.updates += 1
        return True

    def predict(self, phi: np.ndarray) -> np.ndarray:
        phi = np.asarray(phi, dtype=float)
        return phi @ self.theta

    # ------------------------------------------------------------ 状态
    @property
    def ready(self) -> bool:
        """观测标准差降到初值的 1/20 以下即认为已经收敛。"""
        return float(np.trace(self.P)) < self.n * self.delta / 400.0

    def info(self) -> dict:
        return {"updates": self.updates, "skipped": self.skipped,
                "scaled": self.resets, "trace_P": float(np.trace(self.P)),
                "ready": self.ready}

    def state(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.theta.copy(), self.P.copy()

    def load_state(self, theta: np.ndarray, P: Optional[np.ndarray] = None) -> None:
        self.theta = np.asarray(theta, dtype=float).reshape(self.n, self.m).copy()
        self.theta0 = self.theta.copy()          # 泄漏项的锚点
        self._anchored = True
        if P is not None:
            self.P = np.asarray(P, dtype=float).copy()
