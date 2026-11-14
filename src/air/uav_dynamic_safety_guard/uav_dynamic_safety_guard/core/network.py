#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全层网络：观测 + 标称指令 → 修正指令，纯 numpy 前向与反向。

``in_dim`` 由特征类型决定：时序 332+3、单帧 116+3。两者共用同一实现，
因此“单帧 vs 时序”是一次干净的消融，而不是两套代码的差别。
"""
from __future__ import annotations

import json
from typing import Optional, Sequence, Tuple

import numpy as np

from .nn import MLP, Adam

__all__ = ["NetConfig", "GuardNet", "SEQ_IN", "SINGLE_IN"]

SEQ_IN = 332
SINGLE_IN = 116


class NetConfig:
    """网络结构配置（保存到 npz，加载时据此重建）。"""

    fields = ("in_dim", "hidden", "depth", "seed", "out_scale", "tag")

    def __init__(self, in_dim: int = SEQ_IN, hidden: int = 192, depth: int = 2,
                 seed: int = 0, out_scale: float = 1.0,
                 tag: str = "seq") -> None:
        self.in_dim = int(in_dim)
        self.hidden = int(hidden)
        self.depth = int(depth)
        self.seed = int(seed)
        self.out_scale = float(out_scale)
        self.tag = str(tag)

    def to_dict(self) -> dict:
        return {k: getattr(self, k) for k in self.fields}

    def __repr__(self) -> str:
        return "NetConfig(%s)" % self.to_dict()


class GuardNet:
    """``Linear(in,192) → ReLU → Linear(192,192) → ReLU → Linear(192,3)``。"""

    def __init__(self, cfg: Optional[NetConfig] = None):
        self.cfg = cfg or NetConfig()
        rng = np.random.default_rng(self.cfg.seed)
        # 网络输入 = 观测特征(in_dim) + 3 维标称指令
        dims = ([self.cfg.in_dim + 3] + [self.cfg.hidden] * self.cfg.depth
                + [3])
        self.mlp = MLP(dims, rng=rng, out_gain=0.5)
        self.layers = self.mlp.layers
        self.mu = np.zeros(self.cfg.in_dim + 3)
        self.sd = np.ones(self.cfg.in_dim + 3)
        self.out_scale = float(self.cfg.out_scale)

    # ------------------------------------------------------------ 归一化
    def set_norm(self, mu: np.ndarray, sd: np.ndarray) -> None:
        mu = np.asarray(mu, float).reshape(-1)
        sd = np.asarray(sd, float).reshape(-1)
        sd = np.where(sd < 1e-6, 1.0, sd)
        if len(mu) != self.cfg.in_dim + 3:
            raise ValueError("归一化维度 %d 与网络输入 %d 不一致"
                             % (len(mu), self.cfg.in_dim + 3))
        self.mu, self.sd = mu, sd

    def _norm(self, X: np.ndarray) -> np.ndarray:
        return (np.asarray(X, float) - self.mu) / self.sd

    # ------------------------------------------------------------ 前向/反向
    def forward(self, X: np.ndarray, cache: bool = False):
        x = self._norm(X)
        if cache:
            y, masks = self.mlp.forward(x, cache=True)
            return y * self.out_scale, masks
        return self.mlp.forward(x) * self.out_scale

    def backward(self, dy: np.ndarray, masks: Sequence[np.ndarray]) -> None:
        self.mlp.backward(np.asarray(dy, float) * self.out_scale, masks)

    def optimizer(self, lr: float = 2e-3, weight_decay: float = 1e-4,
                  clip: float = 5.0) -> Adam:
        return Adam(self.layers, lr=lr, weight_decay=weight_decay, clip=clip)

    # ------------------------------------------------------------ 存取
    def save(self, path: str, meta: Optional[dict] = None) -> None:
        d = {"cfg": np.asarray(json.dumps(self.cfg.to_dict())),
             "mu": self.mu, "sd": self.sd}
        for i, l in enumerate(self.layers):
            d["W%d" % i] = l.W
            d["b%d" % i] = l.b
        if meta:
            d["meta"] = np.asarray(json.dumps(meta, ensure_ascii=False))
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "GuardNet":
        with np.load(path, allow_pickle=False) as d:
            kw = json.loads(str(d["cfg"]))
            net = cls(NetConfig(**kw))
            net.mu = np.array(d["mu"], float)
            net.sd = np.array(d["sd"], float)
            for i, l in enumerate(net.layers):
                l.W = np.array(d["W%d" % i], float)
                l.b = np.array(d["b%d" % i], float)
        return net

    def n_params(self) -> int:
        return int(self.mlp.n_params())

    def summary(self) -> str:
        return "GuardNet(%s, params=%d)" % (
            "→".join(str(d) for d in self.mlp.dims), self.n_params())


def predict_actions(net: GuardNet, X: np.ndarray,
                    batch: int = 2048) -> np.ndarray:
    out = []
    for i in range(0, len(X), int(batch)):
        out.append(net.forward(X[i:i + batch]))
    return np.concatenate(out, axis=0) if out else np.zeros((0, 3))
