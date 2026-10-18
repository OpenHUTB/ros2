"""
e2e_nn.py —— 端到端 MLP（图像 → 15 维关节角回归）
===================================================
纯 NumPy 实现，结构 256→128→64→15，ReLU 激活，线性输出，MSE 损失。
"""
from __future__ import annotations
import numpy as np


def _relu(x): return np.maximum(0, x)
def _relu_grad(x): return (x > 0).astype(np.float64)


class E2EMLP:
    def __init__(self, in_dim=256, h1=128, h2=64, out_dim=15, seed=42):
        rng = np.random.default_rng(seed)
        self.W1 = rng.normal(0, np.sqrt(2/in_dim), (in_dim, h1))
        self.b1 = np.zeros(h1)
        self.W2 = rng.normal(0, np.sqrt(2/h1), (h1, h2))
        self.b2 = np.zeros(h2)
        self.W3 = rng.normal(0, np.sqrt(2/h2), (h2, out_dim))
        self.b3 = np.zeros(out_dim)
        self.m = 0.9
        self._v = [np.zeros_like(x) for x in (self.W1, self.b1, self.W2, self.b2, self.W3, self.b3)]

    def forward(self, X):
        self._X = X
        self._z1 = X @ self.W1 + self.b1
        self._a1 = _relu(self._z1)
        self._z2 = self._a1 @ self.W2 + self.b2
        self._a2 = _relu(self._z2)
        self._out = self._a2 @ self.W3 + self.b3
        return self._out

    def train_step(self, X, y, lr=0.005):
        N = X.shape[0]
        pred = self.forward(X)
        loss = float(np.mean((pred - y)**2))
        d_out = 2*(pred - y) / N
        dW3 = self._a2.T @ d_out
        db3 = d_out.sum(0)
        da2 = d_out @ self.W3.T
        dz2 = da2 * _relu_grad(self._z2)
        dW2 = self._a1.T @ dz2
        db2 = dz2.sum(0)
        da1 = dz2 @ self.W2.T
        dz1 = da1 * _relu_grad(self._z1)
        dW1 = X.T @ dz1
        db1 = dz1.sum(0)
        grads = [dW1, db1, dW2, db2, dW3, db3]
        params = [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]
        for i, (g, p) in enumerate(zip(grads, params)):
            self._v[i] = self.m * self._v[i] - lr * g
            p += self._v[i]
        return loss

    def predict(self, X):
        return self.forward(X)

    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3)

    def load(self, path):
        d = np.load(path)
        self.W1, self.b1 = d["W1"], d["b1"]
        self.W2, self.b2 = d["W2"], d["b2"]
        self.W3, self.b3 = d["W3"], d["b3"]
