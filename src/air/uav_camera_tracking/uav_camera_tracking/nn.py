# -*- coding: utf-8 -*-
"""nn.py —— 纯 NumPy 实现的多层感知机（ReLU + Adam）。

为什么不引入 PyTorch：课程作业要在任何一台装了 numpy 的机器上复现，
而本模块的网络只有几千个参数，numpy 足够快；同时保留手工实现的梯度、
Adam 更新与损失曲线，便于在文档中解释"网络到底在学什么"。
"""
import numpy as np


class MLP:
    """全连接网络。最后一层为线性输出，由调用方决定是否套 sigmoid。"""

    def __init__(self, sizes, seed=0, lr=1e-3):
        rng = np.random.default_rng(seed)
        self.sizes = list(sizes)
        self.W, self.b = [], []
        for i in range(len(self.sizes) - 1):
            fan_in, fan_out = self.sizes[i], self.sizes[i + 1]
            # He 初始化，适配 ReLU
            self.W.append(rng.normal(0.0, np.sqrt(2.0 / fan_in),
                                     (fan_in, fan_out)))
            self.b.append(np.zeros(fan_out))
        self.lr = lr
        self.mu = None      # 输入标准化参数
        self.sd = None
        self.history = {'loss': [], 'val_loss': []}
        # Adam 状态
        self._mw = [np.zeros_like(w) for w in self.W]
        self._vw = [np.zeros_like(w) for w in self.W]
        self._mb = [np.zeros_like(b) for b in self.b]
        self._vb = [np.zeros_like(b) for b in self.b]
        self._t = 0

    # ---------------- 前向/反向 ----------------
    def _forward(self, X):
        acts, zs = [X], []
        h = X
        last = len(self.W) - 1
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            z = h @ w + b
            zs.append(z)
            h = np.maximum(z, 0.0) if i < last else z
            acts.append(h)
        return acts, zs

    def _backward(self, acts, zs, grad_out):
        gw = [None] * len(self.W)
        gb = [None] * len(self.W)
        g = grad_out
        for i in range(len(self.W) - 1, -1, -1):
            gw[i] = acts[i].T @ g
            gb[i] = g.sum(axis=0)
            if i > 0:
                g = (g @ self.W[i].T) * (zs[i - 1] > 0.0)
        return gw, gb

    def _adam(self, gw, gb, beta1=0.9, beta2=0.999, eps=1e-8):
        self._t += 1
        t = self._t
        for i in range(len(self.W)):
            self._mw[i] = beta1 * self._mw[i] + (1 - beta1) * gw[i]
            self._vw[i] = beta2 * self._vw[i] + (1 - beta2) * (gw[i] ** 2)
            mhat = self._mw[i] / (1 - beta1 ** t)
            vhat = self._vw[i] / (1 - beta2 ** t)
            self.W[i] -= self.lr * mhat / (np.sqrt(vhat) + eps)
            self._mb[i] = beta1 * self._mb[i] + (1 - beta1) * gb[i]
            self._vb[i] = beta2 * self._vb[i] + (1 - beta2) * (gb[i] ** 2)
            mhat_b = self._mb[i] / (1 - beta1 ** t)
            vhat_b = self._vb[i] / (1 - beta2 ** t)
            self.b[i] -= self.lr * mhat_b / (np.sqrt(vhat_b) + eps)

    # ---------------- 推理 ----------------
    def _norm(self, X):
        if self.mu is None:
            return X
        return (X - self.mu) / self.sd

    def predict(self, X):
        X = np.atleast_2d(np.asarray(X, dtype=float))
        acts, _ = self._forward(self._norm(X))
        return acts[-1]

    # ---------------- 训练 ----------------
    def fit(self, X, Y, epochs=60, batch=64, lr=1e-3, val_split=0.15,
            bce_cols=(), reg_mask_col=None, seed=0, verbose=False):
        """训练。

        bce_cols: 用二元交叉熵监督的列（其余列用 MSE）；
        reg_mask_col: 作为回归权重掩码的列（如可见性），不可见样本的回归项
            不参与梯度——否则占位标签会把网络往画面中心拽。
        """
        X = np.asarray(X, dtype=float)
        Y = np.asarray(Y, dtype=float)
        rng = np.random.default_rng(seed)
        n = X.shape[0]
        idx = rng.permutation(n)
        n_val = max(1, int(n * val_split))
        val_idx, tr_idx = idx[:n_val], idx[n_val:]
        self.mu = X[tr_idx].mean(axis=0)
        self.sd = X[tr_idx].std(axis=0) + 1e-6
        Xtr, Ytr = self._norm(X[tr_idx]), Y[tr_idx]
        Xva, Yva = self._norm(X[val_idx]), Y[val_idx]
        self.lr = lr
        self.history = {'loss': [], 'val_loss': []}
        m = Xtr.shape[0]
        for ep in range(epochs):
            order = rng.permutation(m)
            for s in range(0, m, batch):
                bi = order[s:s + batch]
                xb, yb = Xtr[bi], Ytr[bi]
                acts, zs = self._forward(xb)
                out = acts[-1]
                grad = np.zeros_like(out)
                cols = [c for c in range(out.shape[1]) if c not in bce_cols]
                if cols:
                    err = 2.0 * (out[:, cols] - yb[:, cols]) / out.shape[0]
                    if reg_mask_col is not None:
                        err = err * yb[:, reg_mask_col][:, None]
                    grad[:, cols] = err
                for c in bce_cols:
                    p = 1.0 / (1.0 + np.exp(-out[:, c]))
                    grad[:, c] = (p - yb[:, c]) / out.shape[0]
                gw, gb = self._backward(acts, zs, grad)
                self._adam(gw, gb)
            self.history['loss'].append(
                float(self._loss(Xtr, Ytr, bce_cols, reg_mask_col)))
            self.history['val_loss'].append(
                float(self._loss(Xva, Yva, bce_cols, reg_mask_col)))
            if verbose and (ep % 10 == 0 or ep == epochs - 1):
                print('  epoch %3d  loss %.4f  val %.4f'
                      % (ep, self.history['loss'][-1],
                         self.history['val_loss'][-1]))
        return self.history

    def _loss(self, X, Y, bce_cols, reg_mask_col=None):
        out = self._forward(X)[0][-1]
        total = 0.0
        cols = [c for c in range(out.shape[1]) if c not in bce_cols]
        if cols:
            err2 = (out[:, cols] - Y[:, cols]) ** 2
            if reg_mask_col is not None:
                w = Y[:, reg_mask_col][:, None]
                total += float((err2 * w).sum() / max(1.0, w.sum()))
                total *= 1.0
            else:
                total += float(np.mean(err2))
        for c in bce_cols:
            p = np.clip(1.0 / (1.0 + np.exp(-out[:, c])), 1e-6, 1 - 1e-6)
            total += float(-np.mean(Y[:, c] * np.log(p) + (1 - Y[:, c]) * np.log(1 - p)))
        return total

    # ---------------- 存取 ----------------
    def save(self, path):
        data = {'sizes': np.array(self.sizes), 'mu': self.mu, 'sd': self.sd}
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            data['W%d' % i] = w
            data['b%d' % i] = b
        np.savez(path, **data)

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        sizes = [int(x) for x in d['sizes']]
        net = cls(sizes)
        net.mu, net.sd = d['mu'], d['sd']
        net.W = [d['W%d' % i] for i in range(len(sizes) - 1)]
        net.b = [d['b%d' % i] for i in range(len(sizes) - 1)]
        return net
