# -*- coding: utf-8 -*-
"""单隐层 MLP 关键词分类器（纯 numpy 实现，含前向/反向传播与训练循环）。

参数规模约 80*64 + 64*11 ≈ 5.8k，权重 < 50 KB —— 与「不下 ASR 引擎、不下 72 MB
Whisper 权重」的设计目标一致。
"""

import numpy as np

from humanoid_kws_command import config

import humanoid_kws_command.corpus as corpus


class MLP:
    """标准化的 单隐层 ReLU + softmax 分类器。"""

    def __init__(self, n_in, n_hidden=config.HIDDEN, n_out=len(corpus.LABELS),
                 seed=config.SEED):
        rng = np.random.default_rng(seed)
        self.W1 = (rng.normal(0, np.sqrt(2.0 / n_in), (n_in, n_hidden))).astype(np.float32)
        self.b1 = np.zeros(n_hidden, dtype=np.float32)
        self.W2 = (rng.normal(0, np.sqrt(2.0 / n_hidden), (n_hidden, n_out))).astype(np.float32)
        self.b2 = np.zeros(n_out, dtype=np.float32)
        self.mu = None
        self.sd = None

    # ---------------------------------------------------------------- 归一化
    def fit_scaler(self, X):
        self.mu = X.mean(axis=0)
        self.sd = X.std(axis=0) + 1e-6

    def _norm(self, X):
        return (X - self.mu) / self.sd

    # ---------------------------------------------------------------- 前向
    def forward(self, X):
        Z = self._norm(X)
        h = np.maximum(Z @ self.W1 + self.b1, 0.0)
        logits = h @ self.W2 + self.b2
        logits = logits - logits.max(axis=1, keepdims=True)
        exp = np.exp(logits)
        return h, exp / exp.sum(axis=1, keepdims=True)

    def predict(self, X):
        return self.forward(X)[1].argmax(axis=1)

    def proba(self, X):
        return self.forward(X)[1]

    # ---------------------------------------------------------------- 训练
    def fit(self, X, y, epochs=config.EPOCHS, lr=config.LR, l2=config.L2,
            batch=config.BATCH, seed=config.SEED):
        """mini-batch SGD + 动量。返回每轮的训练损失，便于画曲线。"""
        self.fit_scaler(X)
        rng = np.random.default_rng(seed)
        vW1 = np.zeros_like(self.W1); vb1 = np.zeros_like(self.b1)
        vW2 = np.zeros_like(self.W2); vb2 = np.zeros_like(self.b2)
        mom, n = 0.9, len(X)
        Y = np.eye(len(corpus.LABELS), dtype=np.float32)[y]
        hist = []
        for ep in range(epochs):
            order = rng.permutation(n)
            tot = 0.0
            for s in range(0, n, batch):
                idx = order[s:s + batch]
                xb, yb = X[idx], Y[idx]
                h, p = self.forward(xb)
                loss = -np.mean(np.sum(yb * np.log(p + 1e-9), axis=1))
                tot += loss * len(idx)
                dlog = (p - yb) / len(idx)
                gW2 = h.T @ dlog + l2 * self.W2
                gb2 = dlog.sum(axis=0)
                dh = (dlog @ self.W2.T) * (h > 0)
                z = self._norm(xb)
                gW1 = z.T @ dh + l2 * self.W1
                gb1 = dh.sum(axis=0)
                vW1 = mom * vW1 - lr * gW1; self.W1 += vW1
                vb1 = mom * vb1 - lr * gb1; self.b1 += vb1
                vW2 = mom * vW2 - lr * gW2; self.W2 += vW2
                vb2 = mom * vb2 - lr * gb2; self.b2 += vb2
            hist.append(tot / n)
        return hist

    # ---------------------------------------------------------------- 存取
    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 mu=self.mu, sd=self.sd)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        m = cls(d["W1"].shape[0], d["W1"].shape[1])
        m.W1, m.b1, m.W2, m.b2 = d["W1"], d["b1"], d["W2"], d["b2"]
        m.mu, m.sd = d["mu"], d["sd"]
        return m


def params_count(m):
    """可训练参数个数（写入文档，便于和 Whisper tiny 的 39 M 对比）。"""
    return int(m.W1.size + m.b1.size + m.W2.size + m.b2.size)