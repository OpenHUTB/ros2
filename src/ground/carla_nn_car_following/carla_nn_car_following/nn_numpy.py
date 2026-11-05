# -*- coding: utf-8 -*-
"""纯 NumPy 实现的多层感知机（MLP）：前向传播、反向传播、Adam 优化器。

为什么手写而不用 PyTorch/TensorFlow
-----------------------------------
1. **零额外依赖**：Ubuntu 22.04 + ROS 2 Humble 环境里不需要再装几百 MB 的框架，
   别人复现时少一个失败点；
2. **可解释**：手写实现能做到公式与代码逐行对应，每一层的输入输出都可以单独打印核对
   （见文档第 3 章）。

网络结构（与文档 3.3 / 3.4 节一致）::

    输入层 -> 隐藏层1(tanh) -> 隐藏层2(tanh) -> 输出层(线性)

前向传播::

    h1 = tanh(x W1 + b1)
    h2 = tanh(h1 W2 + b2)
    y  = h2 W3 + b3

损失函数（均方误差）与反向传播::

    L = (1/N) * sum_i ||y_i - t_i||^2
    dL/dz_out = 2 (y - t) / N
    dL/dW_l  = a_{l-1}^T delta_l + lambda W_l        (lambda 为 L2 正则系数)
    dL/db_l  = sum(delta_l)
    delta_{l-1} = (delta_l W_l^T) * (1 - a_{l-1}^2)  (tanh 的导数)
"""

import numpy as np


class Normalizer:
    """按特征做标准化：x' = (x - mean) / std。

    神经网络对输入尺度非常敏感（雷达距离是几十米、点云数量是几百个），
    不做标准化会出现训练不收敛。
    """

    def __init__(self, mean=None, std=None):
        self.mean = None if mean is None else np.asarray(mean, dtype=np.float64)
        self.std = None if std is None else np.asarray(std, dtype=np.float64)

    def fit(self, X):
        X = np.asarray(X, dtype=np.float64)
        self.mean = X.mean(axis=0)
        self.std = X.std(axis=0) + 1e-8      # 防止除零
        return self

    def transform(self, X):
        return (np.asarray(X, dtype=np.float64) - self.mean) / self.std

    def inverse(self, Y):
        return np.asarray(Y, dtype=np.float64) * self.std + self.mean


class MLP:
    """多层感知机。"""

    def __init__(self, layer_sizes, seed=0, lr=0.01, l2=1e-5):
        self.layer_sizes = [int(n) for n in layer_sizes]
        self.lr = float(lr)
        self.l2 = float(l2)

        rng = np.random.default_rng(seed)
        self.W = []
        self.b = []
        for n_in, n_out in zip(self.layer_sizes[:-1], self.layer_sizes[1:]):
            # He 初始化：方差 2/n_in，避免深层网络激活值过大或过小
            self.W.append(rng.normal(0.0, np.sqrt(2.0 / n_in), size=(n_in, n_out)))
            self.b.append(np.zeros(n_out, dtype=np.float64))

        self._adam_state = None
        self._adam_step = 0

    # ---------------- 前向 ----------------
    def forward(self, X):
        """返回每一层的激活值列表 ``[a0, a1, ..., y]``，反向传播时复用。"""
        activations = [np.asarray(X, dtype=np.float64)]
        a = activations[0]
        last = len(self.W) - 1
        for index, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W + b
            a = np.tanh(z) if index < last else z
            activations.append(a)
        return activations

    def predict(self, X):
        return self.forward(X)[-1]

    # ---------------- 反向 ----------------
    def _backward(self, activations, target):
        n_samples = target.shape[0]
        grads_W = [None] * len(self.W)
        grads_b = [None] * len(self.b)

        delta = 2.0 * (activations[-1] - target) / n_samples     # dL/dz_out
        for index in range(len(self.W) - 1, -1, -1):
            grads_W[index] = activations[index].T @ delta + self.l2 * self.W[index]
            grads_b[index] = delta.sum(axis=0)
            if index > 0:
                delta = (delta @ self.W[index].T) * (1.0 - activations[index] ** 2)
        return grads_W, grads_b

    def _adam_update(self, grads_W, grads_b, beta1=0.9, beta2=0.999, eps=1e-8):
        if self._adam_state is None:
            self._adam_state = (
                [np.zeros_like(w) for w in self.W], [np.zeros_like(w) for w in self.W],
                [np.zeros_like(b) for b in self.b], [np.zeros_like(b) for b in self.b],
            )
        mW, vW, mb, vb = self._adam_state
        self._adam_step += 1
        t = self._adam_step

        for index in range(len(self.W)):
            mW[index] = beta1 * mW[index] + (1.0 - beta1) * grads_W[index]
            vW[index] = beta2 * vW[index] + (1.0 - beta2) * (grads_W[index] ** 2)
            m_hat = mW[index] / (1.0 - beta1 ** t)
            v_hat = vW[index] / (1.0 - beta2 ** t)
            self.W[index] -= self.lr * m_hat / (np.sqrt(v_hat) + eps)

            mb[index] = beta1 * mb[index] + (1.0 - beta1) * grads_b[index]
            vb[index] = beta2 * vb[index] + (1.0 - beta2) * (grads_b[index] ** 2)
            m_hat = mb[index] / (1.0 - beta1 ** t)
            v_hat = vb[index] / (1.0 - beta2 ** t)
            self.b[index] -= self.lr * m_hat / (np.sqrt(v_hat) + eps)

        self._adam_state = (mW, vW, mb, vb)

    # ---------------- 训练 ----------------
    def fit(self, X, Y, epochs=300, batch_size=64, seed=0, verbose=True, log_every=50):
        """小批量 Adam 训练，返回每个 epoch 的训练集 MSE 曲线。"""
        X = np.asarray(X, dtype=np.float64)
        Y = np.asarray(Y, dtype=np.float64)
        n_samples = X.shape[0]
        rng = np.random.default_rng(seed)
        history = []

        for epoch in range(1, int(epochs) + 1):
            order = rng.permutation(n_samples)
            for start in range(0, n_samples, int(batch_size)):
                batch = order[start:start + int(batch_size)]
                activations = self.forward(X[batch])
                grads_W, grads_b = self._backward(activations, Y[batch])
                self._adam_update(grads_W, grads_b)

            loss = self.mse(self.predict(X), Y)
            history.append(loss)
            if verbose and (epoch == 1 or epoch % int(log_every) == 0):
                print('    epoch %4d  训练 MSE = %.6f' % (epoch, loss))
        return history

    @staticmethod
    def mse(prediction, target):
        diff = np.asarray(prediction) - np.asarray(target)
        return float(np.mean(diff ** 2))

    # ---------------- 保存 / 加载 ----------------
    def save(self, path, meta=None):
        payload = {'layer_sizes': np.array(self.layer_sizes, dtype=np.int64)}
        for index, (W, b) in enumerate(zip(self.W, self.b)):
            payload['W%d' % index] = W
            payload['b%d' % index] = b
        for key, value in (meta or {}).items():
            payload['meta_' + key] = np.asarray(value)
        np.savez(path, **payload)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            sizes = [int(v) for v in data['layer_sizes']]
            model = cls(sizes)
            for index in range(len(sizes) - 1):
                model.W[index] = np.array(data['W%d' % index], dtype=np.float64)
                model.b[index] = np.array(data['b%d' % index], dtype=np.float64)
            meta = {}
            for key in data.files:
                if key.startswith('meta_'):
                    meta[key[len('meta_'):]] = np.array(data[key])
        return model, meta
