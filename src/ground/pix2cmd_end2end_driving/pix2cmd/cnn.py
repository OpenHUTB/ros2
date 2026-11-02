"""A tiny convolutional network implemented with numpy only.

Two stride-2 convolutions followed by two fully connected layers. The last
layer splits into tanh (steering) and sigmoid (throttle). Everything -
including the im2col/col2im convolution backward pass and Adam - is plain
numpy, so the module trains on a CPU without PyTorch or TensorFlow.
"""

from __future__ import annotations

import numpy as np

K = 3
STRIDE = 2


def im2col(x: np.ndarray, stride: int = STRIDE) -> tuple[np.ndarray, tuple]:
    n, h, w, c = x.shape
    ho = (h - K) // stride + 1
    wo = (w - K) // stride + 1
    cols = np.zeros((n, ho, wo, K, K, c), np.float32)
    for i in range(K):
        for j in range(K):
            cols[:, :, :, i, j, :] = x[:, i:i + stride * ho:stride, j:j + stride * wo:stride, :]
    return cols.reshape(n * ho * wo, K * K * c), (x.shape, ho, wo)


def col2im(dcols: np.ndarray, shape: tuple, ho: int, wo: int, stride: int = STRIDE) -> np.ndarray:
    n, h, w, c = shape
    dx = np.zeros(shape, np.float32)
    d = dcols.reshape(n, ho, wo, K, K, c)
    for i in range(K):
        for j in range(K):
            dx[:, i:i + stride * ho:stride, j:j + stride * wo:stride, :] += d[:, :, :, i, j, :]
    return dx


class TinyCNN:
    """Conv(3x3,8,s2) -> ReLU -> Conv(3x3,16,s2) -> ReLU -> FC(32) -> ReLU -> FC(2)."""

    def __init__(self, in_hw: int = 48, c1: int = 8, c2: int = 16, hidden: int = 32, seed: int = 0):
        rng = np.random.default_rng(seed)
        h1 = (in_hw - K) // STRIDE + 1
        h2 = (h1 - K) // STRIDE + 1
        flat = h2 * h2 * c2
        self.shapes = (in_hw, h1, h2, flat)
        self.p = {
            "w1": (rng.normal(0, 0.30, (K, K, 1, c1))).astype(np.float32),
            "b1": np.zeros(c1, np.float32),
            "w2": (rng.normal(0, 0.25, (K, K, c1, c2))).astype(np.float32),
            "b2": np.zeros(c2, np.float32),
            "w3": (rng.normal(0, 0.05, (flat, hidden))).astype(np.float32),
            "b3": np.zeros(hidden, np.float32),
            "w4": (rng.normal(0, 0.05, (hidden, 2))).astype(np.float32),
            "b4": np.zeros(2, np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.p.items()}
        self.t = 0

    # ---------------------------------------------------------------- forward
    def forward(self, x: np.ndarray):
        p = self.p
        n = x.shape[0]
        cols1, info1 = im2col(x)
        h1 = info1[1]
        z1 = cols1 @ p["w1"].reshape(-1, p["w1"].shape[-1]) + p["b1"]
        a1 = np.maximum(z1, 0.0)
        a1v = a1.reshape(n, h1, h1, -1)

        cols2, info2 = im2col(a1v)
        h2 = info2[1]
        z2 = cols2 @ p["w2"].reshape(-1, p["w2"].shape[-1]) + p["b2"]
        a2 = np.maximum(z2, 0.0)

        z3 = a2.reshape(n, -1) @ p["w3"] + p["b3"]
        a2f = a2.reshape(n, -1)
        a3 = np.maximum(z3, 0.0)
        z4 = a3 @ p["w4"] + p["b4"]

        steer = np.tanh(z4[:, 0])
        throttle = 1.0 / (1.0 + np.exp(-z4[:, 1]))
        cache = (x, cols1, info1, z1, a1, cols2, info2, z2, a2f, z3, a3, z4, steer, throttle)
        return np.stack([steer, throttle], axis=1), cache

    # --------------------------------------------------------------- backward
    def backward(self, cache, y_true: np.ndarray) -> dict:
        p = self.p
        x, cols1, info1, z1, a1, cols2, info2, z2, a2f, z3, a3, z4, steer, throttle = cache
        n = x.shape[0]
        h1, h2 = info1[1], info2[1]

        pred = np.stack([steer, throttle], axis=1)
        dpred = 2.0 * (pred - y_true) / n

        # tanh / sigmoid derivatives
        dz4 = np.empty_like(z4)
        dz4[:, 0] = dpred[:, 0] * (1.0 - steer ** 2)
        s = throttle
        dz4[:, 1] = dpred[:, 1] * s * (1.0 - s)

        g = {}
        g["w4"] = a3.T @ dz4
        g["b4"] = dz4.sum(axis=0)
        da3 = dz4 @ p["w4"].T
        dz3 = da3 * (z3 > 0)
        g["w3"] = a2f.T @ dz3
        g["b3"] = dz3.sum(axis=0)
        da2f = dz3 @ p["w3"].T
        da2 = da2f.reshape(z2.shape)

        dz2 = da2 * (z2 > 0)
        g["w2"] = (cols2.T @ dz2).reshape(p["w2"].shape)
        g["b2"] = dz2.sum(axis=0)
        dcols2 = dz2 @ p["w2"].reshape(-1, p["w2"].shape[-1]).T
        da1v = col2im(dcols2, (n, h1, h1, p["w1"].shape[-1]), h2, h2)
        da1 = da1v.reshape(n * h1 * h1, -1)

        dz1 = da1 * (z1 > 0)
        g["w1"] = (cols1.T @ dz1).reshape(p["w1"].shape)
        g["b1"] = dz1.sum(axis=0)
        return g

    # ------------------------------------------------------------------ train
    def step(self, grads: dict, lr: float = 1e-3) -> None:
        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for key in self.p:
            grad = grads[key]
            if key.startswith("b"):
                grad = grad[: self.p[key].shape[0]]
            self.m[key] = b1 * self.m[key] + (1 - b1) * grad
            self.v[key] = b2 * self.v[key] + (1 - b2) * grad ** 2
            mh = self.m[key] / (1 - b1 ** self.t)
            vh = self.v[key] / (1 - b2 ** self.t)
            self.p[key] -= (lr * mh / (np.sqrt(vh) + eps)).astype(np.float32)

    # ---------------------------------------------------------------- predict
    def predict(self, x: np.ndarray, batch: int = 128) -> np.ndarray:
        out = []
        for i in range(0, x.shape[0], batch):
            pred, _ = self.forward(x[i:i + batch])
            out.append(pred)
        return np.concatenate(out, axis=0)

    def config(self) -> dict:
        return {"in_hw": self.shapes[0], "params": int(sum(v.size for v in self.p.values()))}
