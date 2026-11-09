#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 numpy 的全连接层与 Adam。"""
from __future__ import annotations

import math
from typing import List, Sequence

import numpy as np

__all__ = ["Linear", "Adam", "relu", "relu_backward"]


def relu(x: np.ndarray):
    m = x > 0
    return x * m, m


def relu_backward(dy: np.ndarray, mask: np.ndarray) -> np.ndarray:
    return dy * mask


class Linear:
    """``y = x @ W + b``，参数与梯度同序存放。"""

    def __init__(self, n_in: int, n_out: int, rng: np.random.Generator = None,
                 gain: float = 1.0):
        rng = rng or np.random.default_rng(0)
        lim = math.sqrt(6.0 / (n_in + n_out)) * gain
        self.W = rng.uniform(-lim, lim, (n_in, n_out))
        self.b = np.zeros(n_out)
        self.params = [self.W, self.b]
        self.grads = [np.zeros_like(self.W), np.zeros_like(self.b)]
        self._x = None

    def forward(self, x: np.ndarray) -> np.ndarray:
        self._x = x
        return x @ self.W + self.b

    def backward(self, dy: np.ndarray) -> np.ndarray:
        self.grads[0][...] = self._x.T @ dy
        self.grads[1][...] = dy.sum(axis=0)
        return dy @ self.W.T


class Adam:
    def __init__(self, layers: Sequence, lr: float = 2e-3, b1: float = 0.9,
                 b2: float = 0.999, eps: float = 1e-8,
                 weight_decay: float = 0.0, clip: float = 0.0):
        self.layers = list(layers)
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.wd, self.clip = weight_decay, clip
        self.m: List[np.ndarray] = [np.zeros_like(p)
                                    for l in self.layers for p in l.params]
        self.v: List[np.ndarray] = [np.zeros_like(p)
                                    for l in self.layers for p in l.params]
        self.t = 0

    def zero_grad(self) -> None:
        for l in self.layers:
            for g in l.grads:
                g[...] = 0.0

    def step(self) -> float:
        self.t += 1
        gn = 0.0
        flat = [(l.params[i], l.grads[i]) for l in self.layers
                for i in range(len(l.params))]
        for k, (p, g) in enumerate(flat):
            if self.wd:
                g = g + self.wd * p
            gn += float(np.sum(g * g))
            self.m[k] = self.b1 * self.m[k] + (1 - self.b1) * g
            self.v[k] = self.b2 * self.v[k] + (1 - self.b2) * (g * g)
            mh = self.m[k] / (1 - self.b1 ** self.t)
            vh = self.v[k] / (1 - self.b2 ** self.t)
            upd = self.lr * mh / (np.sqrt(vh) + self.eps)
            if self.clip > 0.0:
                np.clip(upd, -self.clip, self.clip, out=upd)
            p -= upd
        return math.sqrt(gn)
