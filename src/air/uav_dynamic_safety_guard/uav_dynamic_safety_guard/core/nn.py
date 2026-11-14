#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 numpy 的全连接层、多层感知机与 Adam。

整个模块只依赖 numpy：训练、推理、评测都可以在没有 GPU 的机器上跑。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["Linear", "MLP", "Adam", "relu", "selu_free_factor"]


def relu(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (激活值, 反向掩码)。"""
    mask = x > 0
    return x * mask, mask


def selu_free_factor() -> float:                     # pragma: no cover - 兼容占位
    return 1.0


class Linear:
    """``y = x @ W + b``；参数与梯度按 [W, b] 顺序存放。"""

    def __init__(self, n_in: int, n_out: int,
                 rng: Optional[np.random.Generator] = None,
                 gain: float = 1.0) -> None:
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / float(n_in + n_out)) * float(gain)
        self.W = rng.uniform(-lim, lim, (n_in, n_out))
        self.b = np.zeros(n_out)
        self.params = [self.W, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.b)]
        self._x: Optional[np.ndarray] = None

    @property
    def n_in(self) -> int:
        return int(self.W.shape[0])

    @property
    def n_out(self) -> int:
        return int(self.W.shape[1])

    def forward(self, x: np.ndarray) -> np.ndarray:
        self._x = x
        return x @ self.W + self.b

    def backward(self, dy: np.ndarray) -> np.ndarray:
        assert self._x is not None
        self.grads[0][...] = self._x.T @ dy
        self.grads[1][...] = dy.sum(axis=0)
        return dy @ self.W.T


class MLP:
    """``in → hidden… → out`` 的 ReLU 多层感知机。"""

    def __init__(self, dims: Sequence[int],
                 rng: Optional[np.random.Generator] = None,
                 out_gain: float = 0.5) -> None:
        dims = [int(d) for d in dims]
        if len(dims) < 2:
            raise ValueError("dims 至少要有输入与输出两层")
        rng = rng or np.random.default_rng(0)
        self.layers: List[Linear] = []
        for i in range(len(dims) - 1):
            gain = out_gain if i == len(dims) - 2 else 1.0
            self.layers.append(Linear(dims[i], dims[i + 1], rng=rng, gain=gain))

    @property
    def dims(self) -> List[int]:
        return [self.layers[0].n_in] + [l.n_out for l in self.layers]

    def forward(self, x: np.ndarray, cache: bool = False):
        masks: List[np.ndarray] = []
        h = np.asarray(x, float)
        last = len(self.layers) - 1
        for i, layer in enumerate(self.layers):
            h = layer.forward(h)
            if i < last:
                h, mask = relu(h)
                masks.append(mask)
        return (h, masks) if cache else h

    def backward(self, dy: np.ndarray, masks: Sequence[np.ndarray]) -> None:
        d = np.asarray(dy, float)
        for i in range(len(self.layers) - 1, -1, -1):
            if i < len(self.layers) - 1:
                d = d * masks[i]
            d = self.layers[i].backward(d)

    def n_params(self) -> int:
        return int(sum(p.size for l in self.layers for p in l.params))


class Adam:
    """Adam + 权重衰减 + 梯度范数裁剪。"""

    def __init__(self, layers: Sequence, lr: float = 2e-3, b1: float = 0.9,
                 b2: float = 0.999, eps: float = 1e-8,
                 weight_decay: float = 0.0, clip: float = 0.0) -> None:
        self.layers = list(layers)
        self.lr = float(lr)
        self.b1, self.b2, self.eps = float(b1), float(b2), float(eps)
        self.wd = float(weight_decay)
        self.clip = float(clip)
        params = [p for l in self.layers for p in l.params]
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def zero_grad(self) -> None:
        for l in self.layers:
            for g in l.grads:
                g[...] = 0.0

    def step(self, lr_scale: float = 1.0) -> float:
        self.t += 1
        flat = [(l.params[i], l.grads[i]) for l in self.layers
                for i in range(len(l.params))]
        gn = 0.0
        for k, (p, g) in enumerate(flat):
            if self.wd:
                g = g + self.wd * p
            gn += float(np.sum(g * g))
            self.m[k] = self.b1 * self.m[k] + (1.0 - self.b1) * g
            self.v[k] = self.b2 * self.v[k] + (1.0 - self.b2) * (g * g)
            mh = self.m[k] / (1.0 - self.b1 ** self.t)
            vh = self.v[k] / (1.0 - self.b2 ** self.t)
            upd = self.lr * float(lr_scale) * mh / (np.sqrt(vh) + self.eps)
            if self.clip > 0.0:
                np.clip(upd, -self.clip, self.clip, out=upd)
            p -= upd
        return math.sqrt(gn)
