"""纯 NumPy 实现的多层感知机（Adam 优化器）。

不依赖 PyTorch/TensorFlow，便于在无 GPU 的虚拟机里训练。
结构：输入 -> [Linear+ReLU] x n -> Linear(2)
损失：均方误差 MSE
"""

import numpy as np


def _he(shape, rng):
    fan_in = shape[0]
    return rng.normal(0.0, np.sqrt(2.0 / fan_in), size=shape).astype(np.float64)


class MLP(object):
    def __init__(self, layer_sizes, seed=0, lr=1e-3, l2=0.0):
        self.rng = np.random.default_rng(seed)
        self.layer_sizes = list(layer_sizes)
        self.lr = float(lr)
        self.l2 = float(l2)
        self.W = []
        self.b = []
        for i in range(len(layer_sizes) - 1):
            self.W.append(_he((layer_sizes[i], layer_sizes[i + 1]), self.rng))
            self.b.append(np.zeros(layer_sizes[i + 1]))

        # Adam 状态
        self.mW = [np.zeros_like(w) for w in self.W]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(bb) for bb in self.b]
        self.vb = [np.zeros_like(bb) for bb in self.b]
        self.t = 0

    # ------------------------------------------------------------------ 前向
    def forward(self, X, cache=False):
        acts = [X]
        h = X
        n = len(self.W)
        for i in range(n):
            z = h @ self.W[i] + self.b[i]
            if i < n - 1:
                h = np.maximum(z, 0.0)
            else:
                h = z
            acts.append(h)
        if cache:
            return h, acts
        return h

    def predict(self, X):
        return self.forward(X, cache=False)

    # ------------------------------------------------------------------ 反向
    def _step(self, X, Y):
        out, acts = self.forward(X, cache=True)
        n = len(self.W)
        bs = X.shape[0]
        grad = 2.0 * (out - Y) / bs

        dW = [None] * n
        db = [None] * n
        for i in range(n - 1, -1, -1):
            a_prev = acts[i]
            dW[i] = a_prev.T @ grad + self.l2 * self.W[i]
            db[i] = grad.sum(axis=0)
            if i > 0:
                grad = (grad @ self.W[i].T) * (acts[i] > 0.0)

        self.t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for i in range(n):
            self.mW[i] = b1 * self.mW[i] + (1 - b1) * dW[i]
            self.vW[i] = b2 * self.vW[i] + (1 - b2) * (dW[i] ** 2)
            mhat = self.mW[i] / (1 - b1 ** self.t)
            vhat = self.vW[i] / (1 - b2 ** self.t)
            self.W[i] -= self.lr * mhat / (np.sqrt(vhat) + eps)

            self.mb[i] = b1 * self.mb[i] + (1 - b1) * db[i]
            self.vb[i] = b2 * self.vb[i] + (1 - b2) * (db[i] ** 2)
            mhat = self.mb[i] / (1 - b1 ** self.t)
            vhat = self.vb[i] / (1 - b2 ** self.t)
            self.b[i] -= self.lr * mhat / (np.sqrt(vhat) + eps)

    # ------------------------------------------------------------------ 训练
    def fit(self, X, Y, epochs=30, batch_size=256, verbose=True, shuffle=True):
        n = X.shape[0]
        hist = []
        for ep in range(epochs):
            idx = self.rng.permutation(n) if shuffle else np.arange(n)
            tot = 0.0
            for k in range(0, n, batch_size):
                sel = idx[k:k + batch_size]
                self._step(X[sel], Y[sel])
                pred = self.predict(X[sel])
                tot += float(np.mean((pred - Y[sel]) ** 2)) * len(sel)
            loss = tot / n
            hist.append(loss)
            if verbose:
                print("epoch %3d  mse=%.6f" % (ep + 1, loss))
        return hist

    # ------------------------------------------------------------------ 存取
    def save(self, path):
        np.savez(path, **{"W%d" % i: w for i, w in enumerate(self.W)},
                 **{"b%d" % i: bb for i, bb in enumerate(self.b)},
                 sizes=np.array(self.layer_sizes))

    @classmethod
    def load(cls, path):
        z = np.load(path)
        sizes = z["sizes"].tolist()
        m = cls(sizes)
        m.W = [z["W%d" % i] for i in range(len(sizes) - 1)]
        m.b = [z["b%d" % i] for i in range(len(sizes) - 1)]
        return m
