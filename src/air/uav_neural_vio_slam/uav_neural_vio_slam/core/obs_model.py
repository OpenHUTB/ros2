#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""学习式观测噪声模型：给每条路标观测预测一个 σ_range / σ_bearing。

这是第三个后端与第一个后端**唯一的差别**。经典 EKF-SLAM 用一组固定的
``R = diag(σ_r², σ_b²)``；但实测的测距噪声强烈依赖距离 —— 立柱在近处横跨
好几个方位波束、测距很准，在 8 m 外可能只有一个波束勉强擦到柱面，误差随
距离平方增长。用一个常数去覆盖整个量程，要么近处过度悲观（白丢信息）、
要么远处过度乐观（把坏观测当真）。

模型在**真值**上训练：数据集里有真值位姿和真值路标，把每条观测和最近的
真值路标配对，残差就是纯粹的传感器误差，不掺任何估计器的自洽性偏差。

网络拟合的是 ``E[log err²]``，不是 ``log σ²``，两者差一个常数
``E[log χ²₁] = ψ(1/2) + log 2 = -1.2703628``（见 ``LOG_CHI2_MEAN``）。
方位残差是重尾的，``E[log err²]`` 拟合的是典型误差而不是标准差，所以训练后
再做一个标量方差标定（``fit`` 里的 ``k``），把预测方差的均值对齐到实测方差。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .nn import Adam, Linear

__all__ = ["ObsModelConfig", "LearnedObsModel", "build_training_data",
           "LOG_CHI2_MEAN"]

#: ``E[log χ²₁] = ψ(1/2) + log 2``（ψ 为 digamma）。
#: ``log σ² = E[log err²] − LOG_CHI2_MEAN``；写成 γ=0.5772 会少补 0.693，
#: σ 系统性低估 1.414 倍（实测标定比 0.69 而不是 1.0）。
LOG_CHI2_MEAN = -1.2703628454614782


@dataclass
class ObsModelConfig:
    hidden: int = 20
    epochs: int = 400
    lr: float = 5e-3
    batch: int = 256
    seed: int = 0
    #: σ 的兜底限幅（米 / 弧度）
    sigma_r_range: Tuple[float, float] = (0.01, 1.0)
    sigma_b_range: Tuple[float, float] = (0.002, 0.5)


def _features(r: np.ndarray, b: np.ndarray) -> np.ndarray:
    """观测几何 → 特征。距离同时给一次项与二次项（噪声随距离近似平方增长）。"""
    r = np.asarray(r, float) / 10.0
    b = np.asarray(b, float)
    return np.stack([r, np.cos(b), np.sin(b), r * r], axis=-1)


class LearnedObsModel:
    def __init__(self, cfg: Optional[ObsModelConfig] = None):
        self.cfg = cfg or ObsModelConfig()
        rng = np.random.default_rng(self.cfg.seed)
        h = self.cfg.hidden
        self.l1 = Linear(4, h, rng=rng)
        self.l2 = Linear(h, h, rng=rng)
        self.l3 = Linear(h, 2, rng=rng)
        self.layers = [self.l1, self.l2, self.l3]
        self.mu = np.zeros(4)
        self.sd = np.ones(4)
        self.trained = False
        #: 方差标定因子（见 ``fit``），未标定时为 1
        self.cal_r = 1.0
        self.cal_b = 1.0

    # ------------------------------------------------------------ 前向
    def _forward(self, x: np.ndarray, cache: bool = False):
        x = (x - self.mu) / self.sd
        a1, m1 = self._relu(self.l1.forward(x))
        a2, m2 = self._relu(self.l2.forward(a1))
        out = self.l3.forward(a2)
        return (out, (x, m1, m2)) if cache else out

    @staticmethod
    def _relu(x):
        m = x > 0
        return x * m, m

    def predict(self, r: float, b: float) -> Tuple[float, float]:
        """返回 (σ_range, σ_bearing)。

        限幅在 ``exp`` 之前：分布外输入会给出很大的 ``log σ²``，直接取指数
        触发 ``OverflowError``。输入同样夹到训练区间。
        """
        if not self.trained:
            return 0.10, 0.030
        r = float(min(max(r, 0.5), 16.0))
        b = float(np.angle(np.exp(1j * b)))
        x = _features(np.array([r]), np.array([b]))
        out = self._forward(x)[0]
        return (self._clamp_sigma(float(out[0]), self.cfg.sigma_r_range,
                                  self.cal_r),
                self._clamp_sigma(float(out[1]), self.cfg.sigma_b_range,
                                  self.cal_b))

    @staticmethod
    def _clamp_sigma(log_mean_sq: float, rng: Tuple[float, float],
                     cal: float = 1.0) -> float:
        """``E[log err²]`` → σ：先补 ``E[log χ²₁]`` 偏置，再乘标定因子。"""
        lo, hi = rng
        v = 0.5 * (log_mean_sq - LOG_CHI2_MEAN)
        return float(math.exp(min(max(v, math.log(lo)), math.log(hi))) * cal)

    def predict_var(self, r: float, b: float) -> Tuple[float, float]:
        sr, sb = self.predict(r, b)
        return sr * sr, sb * sb

    # ------------------------------------------------------------ 训练
    def fit(self, X: np.ndarray, Y: np.ndarray, res: Optional[np.ndarray] = None,
            verbose: bool = False) -> Dict[str, float]:
        """``X`` 为 (N,4) 特征，``Y`` 为 (N,2) 的 ``log(err²)``。

        给了 ``res``（原始残差）就在训练后做方差标定，否则 ``cal_*`` 保持 1。
        """
        self.mu = X.mean(axis=0)
        sd = X.std(axis=0)
        sd[sd < 1e-6] = 1.0
        self.sd = sd
        opt = Adam(self.layers, lr=self.cfg.lr, weight_decay=1e-6)
        n = len(X)
        rng = np.random.default_rng(self.cfg.seed)
        for ep in range(self.cfg.epochs):
            idx = rng.permutation(n)
            tot = 0.0
            for s in range(0, n, self.cfg.batch):
                sel = idx[s:s + self.cfg.batch]
                xb, yb = X[sel], Y[sel]
                out, (x0, m1, m2) = self._forward(xb, cache=True)
                d = out - yb
                tot += float(np.mean(d ** 2))
                opt.zero_grad()
                d_out = 2.0 * d / d.size
                da2 = self.l3.backward(d_out)
                da1 = self.l2.backward(da2 * m2)
                self.l1.backward(da1 * m1)
                opt.step()
            if verbose and (ep % 100 == 0 or ep == self.cfg.epochs - 1):
                print("    obs_model epoch %3d  MSE=%.5f" % (ep, tot / max(n // self.cfg.batch, 1)))
        self.trained = True
        pred = self._forward(X)
        out = {"mse": float(np.mean((pred - Y) ** 2)), "train_n": float(n)}
        if res is not None and len(res) == len(X):
            out.update(self.calibrate(X, np.asarray(res, float)))
        return out

    # ------------------------------------------------------------ 标定
    def calibrate(self, X: np.ndarray, res: np.ndarray) -> Dict[str, float]:
        """把预测方差的均值对齐到实测方差的均值（单标量，每个输出一个）。

        ``E[log err²]`` 对重尾分布给出的是"典型"误差：方位残差在 1 m 附近有
        2.3° 的散布，但大多数样本接近 0，取对数后均值被典型值主导，直接换算
        出的 σ 偏小。用 ``mean(err²)/mean(σ²)`` 一个因子拉回来，实测把方位
        标定比从 0.39 提到 1.0 附近。
        """
        pred = self._forward(X)
        for k, (out_i, lo_hi) in enumerate(
                ((0, self.cfg.sigma_r_range), (1, self.cfg.sigma_b_range))):
            s2 = np.exp(pred[:, out_i] - LOG_CHI2_MEAN)
            obs2 = res[:, k] ** 2
            f = math.sqrt(float(np.mean(obs2)) / max(float(np.mean(s2)), 1e-12))
            f = min(max(f, 0.25), 8.0)
            setattr(self, "cal_r" if k == 0 else "cal_b", f)
        return {"cal_r": self.cal_r, "cal_b": self.cal_b}

    # ------------------------------------------------------------ 存取
    def save(self, path: str) -> None:
        d = {"cfg": np.asarray(json.dumps(
            {"hidden": self.cfg.hidden, "seed": self.cfg.seed})),
            "mu": self.mu, "sd": self.sd,
            "trained": np.asarray(int(self.trained)),
            "cal": np.asarray([self.cal_r, self.cal_b], float)}
        for i, p in enumerate(self.layers):
            d["W%d" % i] = p.W
            d["b%d" % i] = p.b
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "LearnedObsModel":
        with np.load(path, allow_pickle=False) as d:
            kw = json.loads(str(d["cfg"]))
            m = cls(ObsModelConfig(**kw))
            m.mu, m.sd = np.array(d["mu"]), np.array(d["sd"])
            for i, p in enumerate(m.layers):
                p.W = np.array(d["W%d" % i])
                p.b = np.array(d["b%d" % i])
            m.trained = bool(int(d["trained"]))
            if "cal" in d.files:
                m.cal_r, m.cal_b = (float(v) for v in np.array(d["cal"]))
        return m


# --------------------------------------------------------------------------- #
def build_training_data(ds, detector, match_radius: float = 0.6
                        ) -> Dict[str, np.ndarray]:
    """从数据集里算出观测残差（用**真值**位姿与真值路标，与估计器无关）。"""
    gt = np.asarray(ds.landmarks, float)
    feats, res = [], []
    for ep in ds.episodes:
        for t in range(len(ep)):
            x, y = ep.pos[t, 0], ep.pos[t, 1]
            th = float(ep.yaw[t])
            dx = gt[:, 0] - x
            dy = gt[:, 1] - y
            r_true = np.hypot(dx, dy)
            b_true = np.arctan2(dy, dx) - th
            for k in range(int(ep.obs_n[t])):
                r, b = float(ep.obs_r[t, k]), float(ep.obs_b[t, k])
                # 机体系最近邻配对
                ex = r_true * np.cos(b_true) - r * math.cos(b)
                ey = r_true * np.sin(b_true) - r * math.sin(b)
                d = np.hypot(ex, ey)
                j = int(np.argmin(d))
                if d[j] > match_radius:
                    continue
                feats.append([r, b])
                res.append([r - r_true[j],
                            float(np.angle(np.exp(1j * (b - b_true[j]))))])
    F = np.asarray(feats, float)
    R = np.asarray(res, float)
    if len(F) == 0:
        return {"X": np.zeros((0, 4)), "Y": np.zeros((0, 2)), "res": R}
    return {"X": _features(F[:, 0], F[:, 1]),
            "Y": np.log(R ** 2 + 1e-8), "res": R, "geom": F}
