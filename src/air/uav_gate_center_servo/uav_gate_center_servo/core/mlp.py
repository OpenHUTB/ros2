#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""192→32→3 小 MLP：手写前向/反向 + Adam。"""
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

import numpy as np

__all__ = ["TinyMLP"]


class TinyMLP:
    """单隐层 MLP；输入是 12×16 米制深度，输出 [forward,left,up] 米。"""

    def __init__(self, n_in: int = 192, n_hid: int = 32, n_out: int = 3,
                 seed: int = 0):
        self.n_in, self.n_hid, self.n_out = int(n_in), int(n_hid), int(n_out)
        rng = np.random.default_rng(seed)
        lim1 = math.sqrt(6.0 / (self.n_in + self.n_hid))
        lim2 = math.sqrt(6.0 / (self.n_hid + self.n_out))
        self.W1 = rng.uniform(-lim1, lim1, (self.n_in, self.n_hid))
        self.b1 = np.zeros(self.n_hid)
        self.W2 = rng.uniform(-lim2, lim2, (self.n_hid, self.n_out))
        self.b2 = np.zeros(self.n_out)
        self.x_mean = 0.0
        self.x_std = 1.0
        self.y_mean = np.zeros(self.n_out)
        self.y_std = np.ones(self.n_out)
        self._adam = {
            "m1": np.zeros_like(self.W1), "v1": np.zeros_like(self.W1),
            "mb1": np.zeros_like(self.b1), "vb1": np.zeros_like(self.b1),
            "m2": np.zeros_like(self.W2), "v2": np.zeros_like(self.W2),
            "mb2": np.zeros_like(self.b2), "vb2": np.zeros_like(self.b2),
        }
        self._step = 0

    def n_params(self) -> int:
        return int(self.W1.size + self.b1.size + self.W2.size + self.b2.size)

    def summary(self) -> str:
        return "TinyMLP %d->%d->%d (%d params)" % (
            self.n_in, self.n_hid, self.n_out, self.n_params())

    def set_stats(self, depth: np.ndarray, label: np.ndarray) -> None:
        d = np.asarray(depth, np.float32)
        valid = np.isfinite(d) & (d > 0.0)
        vals = d[valid]
        self.x_mean = float(vals.mean()) if vals.size else 0.0
        self.x_std = max(float(vals.std()), 1e-6)
        y = np.asarray(label, np.float32).reshape(-1, self.n_out)
        self.y_mean = y.mean(axis=0)
        self.y_std = np.maximum(y.std(axis=0), 1e-6)

    def _norm_x(self, depth: np.ndarray) -> np.ndarray:
        d = np.asarray(depth, np.float32)
        if d.ndim == 1:
            d = d.reshape(1, -1)
        return ((d - self.x_mean) / self.x_std).reshape(d.shape[0], -1)

    def _norm_y(self, label: np.ndarray) -> np.ndarray:
        y = np.asarray(label, np.float32).reshape(-1, self.n_out)
        return (y - self.y_mean) / self.y_std

    def forward(self, x: np.ndarray, cache: bool = False):
        x = np.asarray(x, np.float32)
        if x.ndim == 1:
            x = x[None, :]
        h = np.tanh(x @ self.W1 + self.b1)
        y = h @ self.W2 + self.b2
        return (y, (x, h)) if cache else y

    def backward(self, dy: np.ndarray, cache) -> Tuple[np.ndarray, ...]:
        x, h = cache
        dy = np.asarray(dy, np.float32)
        gW2 = h.T @ dy
        gb2 = dy.sum(axis=0)
        dh = dy @ self.W2.T
        dz = dh * (1.0 - h * h)
        gW1 = x.T @ dz
        gb1 = dz.sum(axis=0)
        return gW1, gb1, gW2, gb2

    def adam_step(self, grads, lr: float = 1e-3, b1: float = 0.9,
                  b2: float = 0.999, eps: float = 1e-8,
                  weight_decay: float = 0.0) -> None:
        self._step += 1
        t = self._step
        gW1, gb1, gW2, gb2 = grads
        if weight_decay:
            gW1 = gW1 + weight_decay * self.W1
            gW2 = gW2 + weight_decay * self.W2
        pairs = (("W1", gW1, "m1", "v1"), ("b1", gb1, "mb1", "vb1"),
                 ("W2", gW2, "m2", "v2"), ("b2", gb2, "mb2", "vb2"))
        for name, g, mk, vk in pairs:
            p = getattr(self, name)
            m, v = self._adam[mk], self._adam[vk]
            m *= b1
            m += (1.0 - b1) * g
            v *= b2
            v += (1.0 - b2) * (g * g)
            mh = m / (1.0 - b1 ** t)
            vh = v / (1.0 - b2 ** t)
            p -= lr * mh / (np.sqrt(vh) + eps)

    def train_batch(self, depth: np.ndarray, label: np.ndarray,
                    lr: float = 1e-3, weight_decay: float = 1e-4) -> float:
        x = self._norm_x(depth)
        y = self._norm_y(label)
        yh, cache = self.forward(x, cache=True)
        err = yh - y
        loss = float(np.mean(err * err))
        grads = self.backward(2.0 * err / max(err.size, 1), cache)
        self.adam_step(grads, lr=lr, weight_decay=weight_decay)
        return loss

    def predict(self, depth: np.ndarray) -> np.ndarray:
        d = np.asarray(depth, np.float32)
        single = d.ndim == 2
        if single:
            d = d[None, ...]
        yh = self.forward(self._norm_x(d))
        y = yh * self.y_std + self.y_mean
        y[:, 0] = np.clip(y[:, 0], 0.0, 10.0)
        y[:, 1:] = np.clip(y[:, 1:], -3.0, 3.0)
        return y[0].astype(np.float32) if single else y.astype(np.float32)

    def save(self, path: str, meta: Optional[Dict] = None) -> None:
        np.savez_compressed(
            path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
            x_mean=np.asarray(self.x_mean), x_std=np.asarray(self.x_std),
            y_mean=self.y_mean, y_std=self.y_std,
            meta_json=np.asarray(json.dumps(meta or {}, ensure_ascii=False)),
        )

    @classmethod
    def load(cls, path: str) -> "TinyMLP":
        with np.load(path, allow_pickle=False) as d:
            W1 = d["W1"]
            net = cls(n_in=W1.shape[0], n_hid=W1.shape[1], n_out=d["W2"].shape[1])
            net.W1 = np.array(W1, float)
            net.b1 = np.array(d["b1"], float)
            net.W2 = np.array(d["W2"], float)
            net.b2 = np.array(d["b2"], float)
            net.x_mean = float(d["x_mean"])
            net.x_std = float(d["x_std"])
            net.y_mean = np.array(d["y_mean"], float)
            net.y_std = np.array(d["y_std"], float)
            net.meta = json.loads(str(d["meta_json"])) if "meta_json" in d.files else {}
        return net
