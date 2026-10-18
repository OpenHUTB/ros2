"""
nn.py —— 纯 NumPy 实现的多层感知机（MLP）
==========================================

不依赖 PyTorch/TensorFlow，降低安装门槛。
结构：256 (16x16 灰度图 flatten) → 128 → 64 → 5（5 类跌倒方向）。
激活：ReLU；输出层：Softmax；损失：交叉熵；优化器：SGD+动量。
"""

from __future__ import annotations

import numpy as np


def _relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, x)


def _relu_grad(x: np.ndarray) -> np.ndarray:
    return (x > 0.0).astype(np.float64)


def _softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - logits.max(axis=1, keepdims=True)
    exp = np.exp(shifted)
    return exp / exp.sum(axis=1, keepdims=True)


class FallPredictorMLP:
    """3 层 MLP 分类器。"""

    def __init__(self, input_dim: int = 256, hidden1: int = 128,
                 hidden2: int = 64, output_dim: int = 5, seed: int = 42):
        rng = np.random.default_rng(seed)
        # He 初始化
        self.W1 = rng.normal(0, np.sqrt(2.0 / input_dim), (input_dim, hidden1))
        self.b1 = np.zeros(hidden1)
        self.W2 = rng.normal(0, np.sqrt(2.0 / hidden1), (hidden1, hidden2))
        self.b2 = np.zeros(hidden2)
        self.W3 = rng.normal(0, np.sqrt(2.0 / hidden2), (hidden2, output_dim))
        self.b3 = np.zeros(output_dim)

        # 动量
        self._vW1 = np.zeros_like(self.W1)
        self._vb1 = np.zeros_like(self.b1)
        self._vW2 = np.zeros_like(self.W2)
        self._vb2 = np.zeros_like(self.b2)
        self._vW3 = np.zeros_like(self.W3)
        self._vb3 = np.zeros_like(self.b3)
        self.momentum = 0.9

    # ------------------------------------------------------------------
    # 前向
    # ------------------------------------------------------------------
    def forward(self, X: np.ndarray) -> np.ndarray:
        """X: (N, input_dim) → softmax 概率 (N, output_dim)。"""
        self._X = X
        self._z1 = X @ self.W1 + self.b1
        self._a1 = _relu(self._z1)
        self._z2 = self._a1 @ self.W2 + self.b2
        self._a2 = _relu(self._z2)
        self._logits = self._a2 @ self.W3 + self.b3
        return _softmax(self._logits)

    # ------------------------------------------------------------------
    # 训练一步
    # ------------------------------------------------------------------
    def train_step(self, X: np.ndarray, y: np.ndarray, lr: float = 0.01):
        """y: (N,) 整数类别。返回平均损失。"""
        N = X.shape[0]
        probs = self.forward(X)
        # 交叉熵损失
        logp = -np.log(probs[np.arange(N), y] + 1e-12)
        loss = float(logp.mean())

        # 反向传播（输出层）
        dz3 = probs.copy()
        dz3[np.arange(N), y] -= 1.0
        dz3 /= N
        dW3 = self._a2.T @ dz3
        db3 = dz3.sum(axis=0)

        da2 = dz3 @ self.W3.T
        dz2 = da2 * _relu_grad(self._z2)
        dW2 = self._a1.T @ dz2
        db2 = dz2.sum(axis=0)

        da1 = dz2 @ self.W2.T
        dz1 = da1 * _relu_grad(self._z1)
        dW1 = X.T @ dz1
        db1 = dz1.sum(axis=0)

        # SGD + 动量
        for dW, db, vW, vb, W, b in [
            (dW1, db1, self._vW1, self._vb1, self.W1, self.b1),
            (dW2, db2, self._vW2, self._vb2, self.W2, self.b2),
            (dW3, db3, self._vW3, self._vb3, self.W3, self.b3),
        ]:
            pass  # 占位，下面统一更新

        self._vW1 = self.momentum * self._vW1 - lr * dW1
        self._vb1 = self.momentum * self._vb1 - lr * db1
        self.W1 += self._vW1
        self.b1 += self._vb1

        self._vW2 = self.momentum * self._vW2 - lr * dW2
        self._vb2 = self.momentum * self._vb2 - lr * db2
        self.W2 += self._vW2
        self.b2 += self._vb2

        self._vW3 = self.momentum * self._vW3 - lr * dW3
        self._vb3 = self.momentum * self._vb3 - lr * db3
        self.W3 += self._vW3
        self.b3 += self._vb3

        return loss

    # ------------------------------------------------------------------
    # 预测
    # ------------------------------------------------------------------
    def predict(self, X: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """返回 (类别, 概率)。"""
        probs = self.forward(X)
        return probs.argmax(axis=1), probs.max(axis=1)

    # ------------------------------------------------------------------
    # 保存 / 加载
    # ------------------------------------------------------------------
    def save(self, path: str):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3)

    def load(self, path: str):
        data = np.load(path)
        self.W1 = data["W1"]
        self.b1 = data["b1"]
        self.W2 = data["W2"]
        self.b2 = data["b2"]
        self.W3 = data["W3"]
        self.b3 = data["b3"]
