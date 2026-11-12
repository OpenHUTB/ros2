"""nn.py —— 纯 NumPy 多模态二分类网络（图像 256 + IMU 2 → 2 类）

要点：
- 输入标准化（均值/方差）随权重一起存取。图像特征与 IMU 弧度尺度差一个量级，
  不标准化时网络会退化成"永远预测多数类"（实测 specificity=0，acc 恰好等于基率）。
- 优化器用 Adam。原来只有一阶动量、固定 lr=0.01，20 个 epoch 就塌到多数类。
"""
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
        # Adam 状态
        self.m = [np.zeros_like(p) for p in self._params()]
        self.v = [np.zeros_like(p) for p in self._params()]
        self.t = 0
        # 输入标准化参数（训练时由 fit_normalizer 设定）
        self.x_mean = np.zeros(input_dim)
        self.x_std = np.ones(input_dim)

    def _params(self):
        return [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]

    def fit_normalizer(self, X):
        self.x_mean = X.mean(axis=0)
        std = X.std(axis=0)
        # std 设下限：图像里有大量接近恒定的格子（背景像素），std 极小时会把微小的
        # 渲染差异放大成巨大的 z 值，使决策边界对噪声异常敏感。实测未设下限时，
        # 只在 pitch 差 1° 的"站立"输入上，输出就从 0.00 翻到 1.00。
        self.x_std = np.maximum(std, 1e-2)

    def forward(self, x):
        x = (np.asarray(x, dtype=np.float64) - self.x_mean) / self.x_std
        self.z1 = x @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        logits = self.h2 @ self.W3 + self.b3
        e = np.exp(logits - logits.max(axis=-1, keepdims=True))
        return e / e.sum(axis=-1, keepdims=True)

    def train_step(self, x, y, lr=0.001, l2=0.0):
        pred = self.forward(x)
        n = x.shape[0]
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
        self.t += 1
        b1p, b2p, eps = 0.9, 0.999, 1e-8
        corr1 = 1 - b1p**self.t
        corr2 = 1 - b2p**self.t
        for i, (p, g) in enumerate(zip(self._params(),
                                     [d_W1, d_b1, d_W2, d_b2, d_W3, d_b3])):
            if l2 and p.ndim == 2:
                g = g + l2 * p
            self.m[i] = b1p*self.m[i] + (1-b1p)*g
            self.v[i] = b2p*self.v[i] + (1-b2p)*g*g
            mhat = self.m[i] / corr1
            vhat = self.v[i] / corr2
            p -= lr * mhat / (np.sqrt(vhat) + eps)
        return float(-np.mean(np.log(pred[np.arange(n), y] + 1e-8)))

    def predict(self, x):
        return self.forward(x)

    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3, x_mean=self.x_mean, x_std=self.x_std)

    def load(self, path):
        d = np.load(path)
        shapes = {"W1": self.W1.shape, "b1": self.b1.shape, "W2": self.W2.shape,
                  "b2": self.b2.shape, "W3": self.W3.shape, "b3": self.b3.shape}
        for k, shp in shapes.items():
            if k in d and tuple(d[k].shape) != tuple(shp):
                raise ValueError(f"权重 {k} 形状 {d[k].shape} 与网络 {shp} 不一致")
        self.W1, self.b1 = d["W1"], d["b1"]
        self.W2, self.b2 = d["W2"], d["b2"]
        self.W3, self.b3 = d["W3"], d["b3"]
        # 兼容旧权重：无标准化参数时退化为恒等
        self.x_mean = d["x_mean"] if "x_mean" in d else np.zeros(self.W1.shape[0])
        self.x_std = d["x_std"] if "x_std" in d else np.ones(self.W1.shape[0])
        return self
