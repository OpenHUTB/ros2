# -*- coding: utf-8 -*-
"""纯 numpy 实现的两层感知机（二分类），无需 PyTorch/TensorFlow。

网络：9 维归一化特征 -> tanh 隐层 -> 线性输出 -> sigmoid 翻车概率。
训练用带类别权重的二元交叉熵 + Adam，解决正样本稀疏（翻车前样本占比低）的问题。
"""
import numpy as np


def _sigmoid(z):
    return 1.0 / (1.0 + np.exp(-np.clip(z, -30.0, 30.0)))


def _he(shape, rng):
    return rng.normal(0.0, np.sqrt(2.0 / shape[1]), size=shape)


class RiskMLP:
    def __init__(self, input_dim, hidden, seed=42):
        rng = np.random.default_rng(seed)
        self.W1 = _he((hidden, input_dim), rng)
        self.b1 = np.zeros(hidden)
        self.W2 = _he((1, hidden), rng)
        self.b2 = np.zeros(1)
        # Adam 状态
        self._m = [np.zeros_like(p) for p in (self.W1, self.b1, self.W2, self.b2)]
        self._v = [np.zeros_like(p) for p in (self.W1, self.b1, self.W2, self.b2)]
        self._t = 0

    # ---------- 前向 ----------
    def forward(self, X):
        z1 = X @ self.W1.T + self.b1
        a1 = np.tanh(z1)
        logit = a1 @ self.W2.T + self.b2
        return logit[:, 0], a1

    def predict_proba(self, X):
        single = np.asarray(X).ndim == 1
        Xb = np.asarray(X, dtype=np.float64)
        if single:
            Xb = Xb[None, :]
        logit, _ = self.forward(Xb)
        p = _sigmoid(logit)
        return float(p[0]) if single else p

    def predict(self, X, threshold=0.5):
        p = self.predict_proba(X)
        if np.isscalar(p):
            return int(p >= threshold)
        return (p >= threshold).astype(np.int64)

    # ---------- 反向（加权 BCE） ----------
    def train_step(self, X, Y, lr, pos_weight):
        n = X.shape[0]
        logit, a1 = self.forward(X)
        p = _sigmoid(logit)
        w = np.where(Y > 0.5, pos_weight, 1.0)
        d_logit = w * (p - Y) / n
        grad_W2 = (d_logit[:, None] * a1).sum(axis=0, keepdims=True)
        grad_b2 = np.array([d_logit.sum()])
        d_a1 = d_logit[:, None] @ self.W2
        d_z1 = d_a1 * (1.0 - a1 ** 2)
        grad_W1 = d_z1.T @ X
        grad_b1 = d_z1.sum(axis=0)
        # 加权 BCE 数值（仅用于日志）
        eps = 1e-9
        loss = float(np.mean(-w * (Y * np.log(p + eps) + (1.0 - Y) * np.log(1.0 - p + eps)))
                     / max(1e-6, np.mean(w)))
        self._adam([self.W1, self.b1, self.W2, self.b2],
                   [grad_W1, grad_b1, grad_W2, grad_b2], lr)
        return loss

    def _adam(self, params, grads, lr, beta1=0.9, beta2=0.999, eps=1e-8):
        self._t += 1
        for i, (p, g) in enumerate(zip(params, grads)):
            self._m[i] = beta1 * self._m[i] + (1 - beta1) * g
            self._v[i] = beta2 * self._v[i] + (1 - beta2) * (g * g)
            mhat = self._m[i] / (1 - beta1 ** self._t)
            vhat = self._v[i] / (1 - beta2 ** self._t)
            p -= lr * mhat / (np.sqrt(vhat) + eps)

    def fit(self, X, Y, epochs=200, batch=64, lr=0.01, seed=42):
        rng = np.random.default_rng(seed)
        n = X.shape[0]
        pos = float(np.sum(Y))
        neg = max(1.0, n - pos)
        pos_weight = float(np.clip(neg / max(1.0, pos), 1.0, 50.0))
        losses = []
        for _ in range(epochs):
            perm = rng.permutation(n)
            ep = []
            for s in range(0, n, batch):
                idx = perm[s:s + batch]
                ep.append(self.train_step(X[idx], Y[idx], lr, pos_weight))
            losses.append(float(np.mean(ep)))
        return losses

    # ---------- 持久化 ----------
    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2)

    @classmethod
    def load(cls, path):
        z = np.load(path)
        m = cls.__new__(cls)
        m.W1, m.b1 = z["W1"], z["b1"]
        m.W2, m.b2 = z["W2"], z["b2"]
        m._m = [np.zeros_like(p) for p in (m.W1, m.b1, m.W2, m.b2)]
        m._v = [np.zeros_like(p) for p in (m.W1, m.b1, m.W2, m.b2)]
        m._t = 0
        return m


def auc_score(prob, label):
    """ROC 曲线下面积，纯 numpy 实现（Mann-Whitney U 等价式）。"""
    prob = np.asarray(prob, dtype=np.float64)
    label = np.asarray(label).astype(np.int64)
    pos = prob[label == 1]
    neg = prob[label == 0]
    if pos.size == 0 or neg.size == 0:
        return float("nan")
    order = np.argsort(prob)
    ranks = np.empty(prob.size, dtype=np.float64)
    ranks[order] = np.arange(1, prob.size + 1, dtype=np.float64)
    # 处理并列概率
    sorted_prob = prob[order]
    i = 0
    while i < sorted_prob.size:
        j = i + 1
        while j < sorted_prob.size and sorted_prob[j] == sorted_prob[i]:
            j += 1
        if j - i > 1:
            ranks[order[i:j]] = np.mean(ranks[order[i:j]])
        i = j
    rank_pos = ranks[label == 1].sum()
    return float((rank_pos - pos.size * (pos.size + 1) / 2.0) / (pos.size * neg.size))
