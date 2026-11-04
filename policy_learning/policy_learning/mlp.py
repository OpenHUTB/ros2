"""两层 MLP（手写前向/反向 + Adam），输入 443，输出 (v, ω)。"""
from __future__ import annotations

import json
import math
from typing import Dict

import numpy as np

__all__ = ["PolicyMLP"]


class PolicyMLP:
    """443 → 64 → 32 → 2，tanh 隐层，线性输出。"""

    def __init__(self, n_in: int = 443, n_h1: int = 96, n_h2: int = 48,
                 n_out: int = 2, seed: int = 0):
        self.n_in, self.n_h1, self.n_h2, self.n_out = (
            int(n_in), int(n_h1), int(n_h2), int(n_out))
        rng = np.random.default_rng(seed)
        self.W1 = self._init(rng, n_in, n_h1)
        self.b1 = np.zeros(n_h1)
        self.W2 = self._init(rng, n_h1, n_h2)
        self.b2 = np.zeros(n_h2)
        self.W3 = self._init(rng, n_h2, n_out)
        self.b3 = np.zeros(n_out)
        self.x_mean = 0.0
        self.x_std = 1.0
        self.y_mean = np.zeros(n_out)
        self.y_std = np.ones(n_out)
        self._adam = {k: np.zeros_like(getattr(self, k))
                      for k in ("W1", "b1", "W2", "b2", "W3", "b3")}
        self._m = {k: np.zeros_like(v) for k, v in self._adam.items()}
        self._v = {k: np.zeros_like(v) for k, v in self._adam.items()}
        self._step = 0

    @staticmethod
    def _init(rng, fan_in, fan_out):
        lim = math.sqrt(6.0 / (fan_in + fan_out))
        return rng.uniform(-lim, lim, (fan_in, fan_out))

    def n_params(self) -> int:
        return int(sum(getattr(self, k).size
                       for k in ("W1", "b1", "W2", "b2", "W3", "b3")))

    def summary(self) -> str:
        return "PolicyMLP %d-%d-%d-%d (%d params)" % (
            self.n_in, self.n_h1, self.n_h2, self.n_out, self.n_params())

    # ---------- 归一化 ----------
    def set_stats(self, x: np.ndarray, y: np.ndarray) -> None:
        x = np.asarray(x, np.float32)
        self.x_mean = float(x.mean())
        self.x_std = max(float(x.std()), 1e-6)
        y = np.asarray(y, np.float32).reshape(-1, self.n_out)
        self.y_mean = y.mean(axis=0)
        self.y_std = np.maximum(y.std(axis=0), 1e-6)

    def _nx(self, x):
        return ((np.asarray(x, np.float32) - self.x_mean) / self.x_std
                ).reshape(np.shape(x)[0], -1)

    # ---------- 前向 / 反向 ----------
    def forward(self, x, cache: bool = False):
        x = np.asarray(x, np.float32)
        if x.ndim == 1:
            x = x[None, :]
        z1 = x @ self.W1 + self.b1
        h1 = np.tanh(z1)
        z2 = h1 @ self.W2 + self.b2
        h2 = np.tanh(z2)
        y = h2 @ self.W3 + self.b3
        return (y, (x, h1, h2)) if cache else y

    def backward(self, dy, cache):
        x, h1, h2 = cache
        gW3 = h2.T @ dy
        gb3 = dy.sum(axis=0)
        dh2 = (dy @ self.W3.T) * (1.0 - h2 * h2)
        gW2 = h1.T @ dh2
        gb2 = dh2.sum(axis=0)
        dh1 = (dh2 @ self.W2.T) * (1.0 - h1 * h1)
        gW1 = x.T @ dh1
        gb1 = dh1.sum(axis=0)
        return gW1, gb1, gW2, gb2, gW3, gb3

    def adam_step(self, grads, lr=1e-3, b1=0.9, b2=0.999, eps=1e-8,
                  weight_decay=0.0):
        self._step += 1
        t = self._step
        for name, g in zip(("W1", "b1", "W2", "b2", "W3", "b3"), grads):
            if weight_decay and name.startswith("W"):
                g = g + weight_decay * getattr(self, name)
            m, v = self._m[name], self._v[name]
            m *= b1
            m += (1.0 - b1) * g
            v *= b2
            v += (1.0 - b2) * (g * g)
            mh = m / (1.0 - b1 ** t)
            vh = v / (1.0 - b2 ** t)
            getattr(self, name)[...] -= lr * mh / (np.sqrt(vh) + eps)

    def train_batch(self, x, y, lr=1e-3, weight_decay=1e-5) -> float:
        xn = self._nx(x)
        yn = (np.asarray(y, np.float32).reshape(-1, self.n_out)
              - self.y_mean) / self.y_std
        yh, cache = self.forward(xn, cache=True)
        err = yh - yn
        loss = float(np.mean(err * err))
        self.adam_step(self.backward(2.0 * err / max(err.size, 1), cache),
                       lr=lr, weight_decay=weight_decay)
        return loss

    def predict(self, x) -> np.ndarray:
        x = np.asarray(x, np.float32)
        single = (x.ndim == 1) or (x.ndim == 2 and x.shape[-1] == self.n_in
                                   and x.shape[0] == 1)
        xx = x[None, :] if x.ndim == 1 else x
        y = self.forward(self._nx(xx)) * self.y_std + self.y_mean
        y[:, 0] = np.clip(y[:, 0], -1.5, 3.0)        # 前向偏移
        y[:, 1] = np.clip(y[:, 1], -2.0, 2.0)        # 左向偏移
        return y[0].astype(np.float32) if single else y.astype(np.float32)

    # ---------- 存取 ----------
    def save(self, path: str, meta: Dict | None = None) -> None:
        np.savez_compressed(
            path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
            W3=self.W3, b3=self.b3,
            x_mean=np.asarray(self.x_mean), x_std=np.asarray(self.x_std),
            y_mean=self.y_mean, y_std=self.y_std,
            meta_json=np.asarray(json.dumps(meta or {}, ensure_ascii=False)))

    @classmethod
    def load(cls, path: str) -> "PolicyMLP":
        with np.load(path, allow_pickle=False) as d:
            net = cls(n_in=d["W1"].shape[0], n_h1=d["W1"].shape[1],
                      n_h2=d["W2"].shape[1], n_out=d["W3"].shape[1])
            for k in ("W1", "b1", "W2", "b2", "W3", "b3"):
                setattr(net, k, np.array(d[k], float))
            net.x_mean = float(d["x_mean"])
            net.x_std = float(d["x_std"])
            net.y_mean = np.array(d["y_mean"], float)
            net.y_std = np.array(d["y_std"], float)
            net.meta = (json.loads(str(d["meta_json"]))
                        if "meta_json" in d.files else {})
        return net
