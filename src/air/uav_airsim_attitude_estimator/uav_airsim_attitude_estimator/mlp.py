"""纯 NumPy 的三层 MLP，用于 IMU 姿态角回归。

结构：30 -> 32 -> 2，tanh 隐层 + 线性输出。标签在训练时做 z-score 标准化，
输出层用线性是为了避免姿态角集中在 0 附近时 tanh 饱和、梯度消失。
"""
from __future__ import annotations

import numpy as np


class MLP:
    def __init__(self, n_in=30, hidden=(32,), n_out=2, seed=0):
        rng = np.random.RandomState(seed)
        dims = (n_in,) + tuple(hidden) + (n_out,)
        self.W, self.b = [], []
        for i in range(len(dims) - 1):
            scale = np.sqrt(2.0 / dims[i])
            self.W.append((rng.randn(dims[i], dims[i + 1]) * scale).astype(np.float32))
            self.b.append(np.zeros(dims[i + 1], dtype=np.float32))
        self.cache = []

    def forward(self, x):
        self.cache = [x]
        h = x
        for i in range(len(self.W)):
            h = h @ self.W[i] + self.b[i]
            if i < len(self.W) - 1:
                h = np.tanh(h)
            self.cache.append(h)
        return h

    def predict(self, x):
        return self.forward(x.astype(np.float32))

    def backward(self, dout):
        """dout: 对线性输出的梯度（输出层为线性）。"""
        grads_W, grads_b = [], []
        d = dout
        for i in range(len(self.W) - 1, -1, -1):
            h_prev = self.cache[i]
            grads_W.append(np.outer(h_prev, d).astype(np.float32))
            grads_b.append(d.astype(np.float32))
            d = d @ self.W[i].T
            if i > 0:
                d = d * (1.0 - self.cache[i] ** 2)
        grads_W.reverse()
        grads_b.reverse()
        return grads_W, grads_b

    def params(self):
        out = {}
        for i, w in enumerate(self.W):
            out["W%d" % i] = w
        for i, b in enumerate(self.b):
            out["b%d" % i] = b
        return out

    def load_state(self, state):
        for i in range(len(self.W)):
            self.W[i] = state["W%d" % i].astype(np.float32)
            self.b[i] = state["b%d" % i].astype(np.float32)

    def save(self, path):
        np.savez_compressed(path, **self.params())

    def load(self, path):
        d = np.load(path)
        self.load_state({k: d[k] for k in d.files})


class Adam:
    def __init__(self, net, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        self.net = net
        self.lr, self.b1, self.b2, self.eps = lr, beta1, beta2, eps
        self.t = 0
        self.mW = [np.zeros_like(w) for w in net.W]
        self.vW = [np.zeros_like(w) for w in net.W]
        self.mb = [np.zeros_like(b) for b in net.b]
        self.vb = [np.zeros_like(b) for b in net.b]

    def step(self, grads_W, grads_b):
        self.t += 1
        for i in range(len(self.net.W)):
            self.mW[i] = self.b1 * self.mW[i] + (1 - self.b1) * grads_W[i]
            self.vW[i] = self.b2 * self.vW[i] + (1 - self.b2) * grads_W[i] ** 2
            mh = self.mW[i] / (1 - self.b1 ** self.t)
            vh = self.vW[i] / (1 - self.b2 ** self.t)
            self.net.W[i] -= self.lr * mh / (np.sqrt(vh) + self.eps)

            self.mb[i] = self.b1 * self.mb[i] + (1 - self.b1) * grads_b[i]
            self.vb[i] = self.b2 * self.vb[i] + (1 - self.b2) * grads_b[i] ** 2
            mh = self.mb[i] / (1 - self.b1 ** self.t)
            vh = self.vb[i] / (1 - self.b2 ** self.t)
            self.net.b[i] -= self.lr * mh / (np.sqrt(vh) + self.eps)
