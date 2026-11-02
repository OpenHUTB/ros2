"""nn.py —— 纯NumPy多模态二分类网络（图像256+IMU2→2类）"""
from __future__ import annotations
import numpy as np


class FallWarningMLP:
    def __init__(self, input_dim=258, hidden1=128, hidden2=64, output_dim=2, seed=0):
        rng = np.random.default_rng(seed)
        self.W1 = rng.normal(0, np.sqrt(2.0/input_dim), (input_dim, hidden1))
        self.b1 = np.zeros(hidden1)
        self.W2 = rng.normal(0, np.sqrt(2.0/hidden1), (hidden1, hidden2))
        self.b2 = np.zeros(hidden2)
        self.W3 = rng.normal(0, np.sqrt(2.0/hidden2), (hidden2, output_dim))
        self.b3 = np.zeros(output_dim)
        self.m = [np.zeros_like(p) for p in self._params()]

    def _params(self):
        return [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]

    def forward(self, x):
        self.z1 = x @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        logits = self.h2 @ self.W3 + self.b3
        # softmax
        e = np.exp(logits - logits.max(axis=-1, keepdims=True))
        return e / e.sum(axis=-1, keepdims=True)

    def train_step(self, x, y, lr=0.01):
        pred = self.forward(x)
        n = x.shape[0]
        # cross-entropy gradient
        d_logits = pred.copy()
        d_logits[np.arange(n), y] -= 1
        d_logits /= n
        d_W3 = self.h2.T @ d_logits; d_b3 = d_logits.sum(0)
        d_h2 = d_logits @ self.W3.T
        d_z2 = d_h2 * (self.z2 > 0)
        d_W2 = self.h1.T @ d_z2; d_b2 = d_z2.sum(0)
        d_h1 = d_z2 @ self.W2.T
        d_z1 = d_h1 * (self.z1 > 0)
        d_W1 = x.T @ d_z1; d_b1 = d_z1.sum(0)
        for i, (p, g) in enumerate(zip(self._params(), [d_W1, d_b1, d_W2, d_b2, d_W3, d_b3])):
            self.m[i] = 0.9*self.m[i] + 0.1*g
            p -= lr * self.m[i]
        return float(-np.mean(np.log(pred[np.arange(n), y] + 1e-8)))

    def predict(self, x):
        return self.forward(x)

    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2, W3=self.W3, b3=self.b3)

    def load(self, path):
        d = np.load(path)
        self.W1, self.b1 = d["W1"], d["b1"]
        self.W2, self.b2 = d["W2"], d["b2"]
        self.W3, self.b3 = d["W3"], d["b3"]
