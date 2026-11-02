#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全网络：共享主干 + 当前间距头 h + 一步预测头 q。

``q(x, u)`` 输入当前观测与候选动作，输出执行该动作一步后的间距预测。
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .nn import Adam, Linear, relu
from .ranges import FEAT_DIM

__all__ = ["NetConfig", "SafetyNet", "PolicyNet"]


@dataclass
class NetConfig:
    hidden: int = 128
    action_dim: int = 5          # 指令 3 维 + 该方向上的激光距离 2 维
    seed: int = 0


class SafetyNet:
    def __init__(self, cfg: Optional[NetConfig] = None):
        self.cfg = cfg or NetConfig()
        rng = np.random.default_rng(self.cfg.seed)
        h = self.cfg.hidden
        self.l1 = Linear(FEAT_DIM, h, rng=rng)
        self.l2 = Linear(h, h, rng=rng)
        self.lh = Linear(h, 1, rng=rng, gain=0.5)
        self.lq1 = Linear(h + self.cfg.action_dim, h, rng=rng)
        self.lq2 = Linear(h, 1, rng=rng, gain=0.5)
        self.layers = [self.l1, self.l2, self.lh, self.lq1, self.lq2]
        self.mu = np.zeros(FEAT_DIM)
        self.sd = np.ones(FEAT_DIM)

    # ------------------------------------------------------------ 前向
    def set_norm(self, mu: np.ndarray, sd: np.ndarray) -> None:
        self.mu = np.asarray(mu, float).copy()
        sd = np.asarray(sd, float).copy()
        sd[sd < 1e-6] = 1.0
        self.sd = sd

    def _trunk(self, X: np.ndarray):
        x = (np.asarray(X, float) - self.mu) / self.sd
        z1 = self.l1.forward(x)
        a1, m1 = relu(z1)
        z2 = self.l2.forward(a1)
        f, m2 = relu(z2)
        return f, (x, m1, m2)

    def features(self, X: np.ndarray) -> np.ndarray:
        return self._trunk(X)[0]

    def h_from_feat(self, feat: np.ndarray) -> np.ndarray:
        return self.lh.forward(feat)[:, 0]

    def q_from_feat(self, feat: np.ndarray, actions: np.ndarray) -> np.ndarray:
        f = np.asarray(feat, float)
        if f.ndim == 1:
            f = f[None, :]
        a = np.asarray(actions, float)
        if a.ndim == 1:
            a = a[None, :]
        if len(f) != len(a):
            f = np.repeat(f, len(a), axis=0)
        qin = np.concatenate([f, a], axis=1)
        zq = self.lq1.forward(qin)
        aq, _ = relu(zq)
        return self.lq2.forward(aq)[:, 0]

    def forward(self, X: np.ndarray, actions: np.ndarray):
        f, cache = self._trunk(X)
        h = self.lh.forward(f)[:, 0]
        qin = np.concatenate([f, np.asarray(actions, float)], axis=1)
        zq = self.lq1.forward(qin)
        aq, mq = relu(zq)
        q = self.lq2.forward(aq)[:, 0]
        return h, q, cache + (f, qin, mq)

    def backward(self, dh: np.ndarray, dq: np.ndarray, cache) -> None:
        x, m1, m2, f, qin, mq = cache
        df = self.lh.backward(np.asarray(dh, float)[:, None])
        dzq = self.lq2.backward(np.asarray(dq, float)[:, None]) * mq
        dqin = self.lq1.backward(dzq)
        df = df + dqin[:, :self.cfg.hidden]
        dz2 = df * m2
        da1 = self.l2.backward(dz2)
        dz1 = da1 * m1
        self.l1.backward(dz1)

    # ------------------------------------------------------------ 训练/存取
    def optimizer(self, lr: float = 2e-3, weight_decay: float = 1e-6,
                  clip: float = 5.0) -> Adam:
        return Adam(self.layers, lr=lr, weight_decay=weight_decay, clip=clip)

    def predict(self, X: np.ndarray) -> np.ndarray:
        return self.h_from_feat(self.features(X))

    def predict_q(self, X: np.ndarray, actions: np.ndarray) -> np.ndarray:
        return self.q_from_feat(self.features(X), actions)

    def save(self, path: str, meta: Optional[dict] = None) -> None:
        d = {"cfg": np.asarray(json.dumps({"hidden": self.cfg.hidden,
                                           "seed": self.cfg.seed})),
             "mu": self.mu, "sd": self.sd}
        for i, l in enumerate(self.layers):
            d["W%d" % i] = l.W
            d["b%d" % i] = l.b
        if meta:
            d["meta"] = np.asarray(json.dumps(meta, ensure_ascii=False))
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "SafetyNet":
        with np.load(path, allow_pickle=False) as d:
            kw = json.loads(str(d["cfg"]))
            net = cls(NetConfig(**kw))
            net.mu, net.sd = np.array(d["mu"]), np.array(d["sd"])
            for i, l in enumerate(net.layers):
                l.W = np.array(d["W%d" % i])
                l.b = np.array(d["b%d" % i])
        return net

    def summary(self) -> str:
        n = sum(p.size for l in self.layers for p in l.params)
        return "SafetyNet(hidden=%d, params=%d)" % (self.cfg.hidden, n)


class PolicyNet:
    """把 (观测, 标称指令) 映射到修正后的指令，模仿几何安全层。"""

    def __init__(self, hidden: int = 128, seed: int = 0):
        rng = np.random.default_rng(seed)
        self.l1 = Linear(FEAT_DIM + 3, hidden, rng=rng)
        self.l2 = Linear(hidden, hidden, rng=rng)
        self.l3 = Linear(hidden, 3, rng=rng, gain=0.5)
        self.layers = [self.l1, self.l2, self.l3]
        self.mu = np.zeros(FEAT_DIM + 3)
        self.sd = np.ones(FEAT_DIM + 3)

    def _norm(self, X):
        return (np.asarray(X, float) - self.mu) / self.sd

    def forward(self, X, cache: bool = False):
        x = self._norm(X)
        z1 = self.l1.forward(x)
        a1, m1 = relu(z1)
        z2 = self.l2.forward(a1)
        a2, m2 = relu(z2)
        y = self.l3.forward(a2)
        return (y, (m1, m2)) if cache else y

    def backward(self, dy, cache) -> None:
        m1, m2 = cache
        da2 = self.l3.backward(dy)
        da1 = self.l2.backward(da2 * m2)
        self.l1.backward(da1 * m1)

    def optimizer(self, lr: float = 2e-3, weight_decay: float = 1e-6) -> Adam:
        return Adam(self.layers, lr=lr, weight_decay=weight_decay, clip=5.0)

    def set_norm(self, mu, sd) -> None:
        self.mu = np.asarray(mu, float).copy()
        sd = np.asarray(sd, float).copy()
        sd[sd < 1e-6] = 1.0
        self.sd = sd

    def save(self, path: str, meta: dict = None) -> None:
        d = {"mu": self.mu, "sd": self.sd,
             "hidden": np.asarray(self.l1.W.shape[1])}
        for i, l in enumerate(self.layers):
            d["W%d" % i] = l.W
            d["b%d" % i] = l.b
        if meta:
            d["meta"] = np.asarray(json.dumps(meta, ensure_ascii=False))
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "PolicyNet":
        with np.load(path, allow_pickle=False) as d:
            net = cls(hidden=int(d["hidden"]))
            net.mu, net.sd = np.array(d["mu"]), np.array(d["sd"])
            for i, l in enumerate(net.layers):
                l.W = np.array(d["W%d" % i])
                l.b = np.array(d["b%d" % i])
        return net

    def summary(self) -> str:
        n = sum(p.size for l in self.layers for p in l.params)
        return "PolicyNet(params=%d)" % n
