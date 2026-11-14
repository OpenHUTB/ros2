"""策略网络：26→64→32→2 的 MLP，纯 numpy 手写前向/反向，够小到能塞进控制回路。

输入 26 维：24 束归一化雷达 + 当前速度 + 上一帧角速度。
输出 2 维：目标速度（sigmoid 缩到 [0, 1.4] m/s）与目标角速度（tanh 缩到 ±1.6 rad/s）。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

IN_DIM = 26
H1, H2 = 64, 32
V_MAX_MODEL, W_MAX_MODEL = 1.4, 1.6


def _xavier(rng: np.random.Generator, n_in: int, n_out: int) -> np.ndarray:
    lim = np.sqrt(6.0 / (n_in + n_out))
    return rng.uniform(-lim, lim, (n_in, n_out)).astype(np.float32)


class LatencyNet:
    def __init__(self, seed: int = 0):
        rng = np.random.default_rng(seed)
        self.p = {
            "w1": _xavier(rng, IN_DIM, H1), "b1": np.zeros(H1, dtype=np.float32),
            "w2": _xavier(rng, H1, H2), "b2": np.zeros(H2, dtype=np.float32),
            "w3": _xavier(rng, H2, 2), "b3": np.zeros(2, dtype=np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.t = 0

    # -------------------------------------------------------------- 前向
    def forward(self, x: np.ndarray) -> tuple[np.ndarray, dict]:
        x = np.atleast_2d(x).astype(np.float32)
        z1 = x @ self.p["w1"] + self.p["b1"]
        h1 = np.tanh(z1)
        z2 = h1 @ self.p["w2"] + self.p["b2"]
        h2 = np.tanh(z2)
        z3 = h2 @ self.p["w3"] + self.p["b3"]
        out = np.stack([V_MAX_MODEL / (1.0 + np.exp(-z3[:, 0])),
                        W_MAX_MODEL * np.tanh(z3[:, 1])], axis=1)
        cache = {"x": x, "h1": h1, "h2": h2, "z3": z3}
        return out, cache

    # -------------------------------------------------------------- 反向
    def backward(self, cache: dict, target: np.ndarray) -> dict:
        x, h1, h2 = cache["x"], cache["h1"], cache["h2"]
        pred, _ = self.forward(x)
        n = x.shape[0]
        g = {}

        # 输出层：sigmoid / tanh 各自的导数
        d3 = np.empty_like(pred)
        d3[:, 0] = 2.0 * (pred[:, 0] - target[:, 0]) / n * pred[:, 0] \
            * (1.0 - pred[:, 0] / V_MAX_MODEL)
        d3[:, 1] = 2.0 * (pred[:, 1] - target[:, 1]) / n * W_MAX_MODEL \
            * (1.0 - (pred[:, 1] / W_MAX_MODEL) ** 2)
        g["w3"] = h2.T @ d3
        g["b3"] = d3.sum(axis=0)
        d2 = (d3 @ self.p["w3"].T) * (1.0 - h2**2)
        g["w2"] = h1.T @ d2
        g["b2"] = d2.sum(axis=0)
        d1 = (d2 @ self.p["w2"].T) * (1.0 - h1**2)
        g["w1"] = x.T @ d1
        g["b1"] = d1.sum(axis=0)
        return {k: v.astype(np.float32) for k, v in g.items()}

    # -------------------------------------------------------------- 优化
    def adam(self, grads: dict, lr: float = 1e-3, b1: float = 0.9, b2: float = 0.999,
             eps: float = 1e-8) -> None:
        self.t += 1
        for k, gk in grads.items():
            self.m[k] = b1 * self.m[k] + (1 - b1) * gk
            self.v[k] = b2 * self.v[k] + (1 - b2) * gk**2
            mh = self.m[k] / (1 - b1**self.t)
            vh = self.v[k] / (1 - b2**self.t)
            self.p[k] -= (lr * mh / (np.sqrt(vh) + eps)).astype(np.float32)

    def fit(self, x: np.ndarray, y: np.ndarray, epochs: int = 120, batch: int = 128,
            lr: float = 2e-3, seed: int = 0) -> list[float]:
        rng = np.random.default_rng(seed)
        hist = []
        for _ in range(epochs):
            order = rng.permutation(len(x))
            tot = 0.0
            for s in range(0, len(x), batch):
                idx = order[s:s + batch]
                _, cache = self.forward(x[idx])
                self.adam(self.backward(cache, y[idx]), lr=lr)
                pred, _ = self.forward(x[idx])
                tot += float(np.mean((pred - y[idx]) ** 2)) * len(idx)
            hist.append(tot / len(x))
        return hist

    # -------------------------------------------------------------- 策略接口
    def act(self, feat: np.ndarray) -> tuple[float, float]:
        out, _ = self.forward(feat)
        return float(out[0, 0]), float(out[0, 1])

    # -------------------------------------------------------------- 存取
    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **self.p)

    def load(self, path: str | Path) -> "LatencyNet":
        with np.load(path) as z:
            for k in self.p:
                self.p[k] = z[k].astype(np.float32)
        return self

    def n_params(self) -> int:
        return int(sum(v.size for v in self.p.values()))
