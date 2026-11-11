#!/usr/bin/env python3
"""
轻量级 NumPy 神经网络训练核心 (无 PyTorch 依赖)
===============================================
为课程作业中的三个神经网络提供统一的训练/推理实现：
  - 轨迹跟踪策略 MLP (10 -> 64 -> 64 -> 4, tanh)
  - Neural A* 引导场 MLP (15 -> 64 -> 32 -> 1, sigmoid)
  - 局部避障策略 MLP (76 -> 128 -> 64 -> 3, tanh)

只依赖 numpy，保证在 Ubuntu 20.04/22.04 + ROS Noetic/Humble 的裸环境可复现；
训练脚本与 ROS 运行时共用同一套前向公式，避免“训练一套、推理另一套”。

数学：
  前向:  a_0 = x
         z_l = a_{l-1} W_l + b_l
         a_l = f_l(z_l)
  损失:  L = mean((a_L - y)^2)
  反向:  delta_L = 2 (a_L - y) * f'(z_L) / (N * D)
         dW_l = a_{l-1}^T delta_l,  db_l = sum(delta_l)
         delta_{l-1} = (delta_l W_l^T) * f'(z_{l-1})
  优化:  Adam (Kingma & Ba, 2015)
"""

import numpy as np


# ──────────────────────────────────────────────────────────────
# 激活函数
# ──────────────────────────────────────────────────────────────

def _act_forward(name, z):
    if name == 'tanh':
        return np.tanh(z)
    if name == 'relu':
        return np.maximum(z, 0.0)
    if name == 'sigmoid':
        return 1.0 / (1.0 + np.exp(-np.clip(z, -60.0, 60.0)))
    if name == 'linear':
        return z
    raise ValueError(f"未知激活函数: {name}")


def _act_backward(name, z, a):
    if name == 'tanh':
        return 1.0 - a * a
    if name == 'relu':
        return (z > 0.0).astype(z.dtype)
    if name == 'sigmoid':
        return a * (1.0 - a)
    if name == 'linear':
        return np.ones_like(z)
    raise ValueError(f"未知激活函数: {name}")


# ──────────────────────────────────────────────────────────────
# MLP
# ──────────────────────────────────────────────────────────────

class NumpyMLP:
    """多层感知机 (全连接)，支持 tanh/relu 隐藏层与多种输出激活"""

    def __init__(self, sizes, hidden_act='tanh', output_act='tanh',
                 seed=0, weights=None):
        self.sizes = list(sizes)
        self.hidden_act = hidden_act
        self.output_act = output_act
        self.rng = np.random.default_rng(seed)

        if weights is not None:
            self.W = [np.asarray(w, dtype=np.float64) for w in weights['W']]
            self.b = [np.asarray(b, dtype=np.float64) for b in weights['b']]
            return

        self.W = []
        self.b = []
        for i in range(len(self.sizes) - 1):
            n_in, n_out = self.sizes[i], self.sizes[i + 1]
            # Xavier 初始化，tanh 网络收敛更稳
            limit = np.sqrt(6.0 / (n_in + n_out))
            self.W.append(self.rng.uniform(-limit, limit, (n_in, n_out)))
            self.b.append(np.zeros(n_out))

    # ── 前向 ──────────────────────────────────────────────
    def forward(self, x):
        x = np.asarray(x, dtype=np.float64)
        a = x
        self._cache_a = [a]
        self._cache_z = []
        n_layers = len(self.W)
        for i in range(n_layers):
            z = a @ self.W[i] + self.b[i]
            act = self.output_act if i == n_layers - 1 else self.hidden_act
            a = _act_forward(act, z)
            self._cache_z.append(z)
            self._cache_a.append(a)
        return a

    __call__ = forward

    # ── 损失与反向 ────────────────────────────────────────
    @staticmethod
    def mse(y_pred, y_true):
        return float(np.mean((y_pred - y_true) ** 2))

    def backward(self, y_true):
        """在最近一次 forward 之后调用，返回梯度列表 [(dW, db), ...]"""
        y_true = np.asarray(y_true, dtype=np.float64)
        y_pred = self._cache_a[-1]
        n = y_pred.shape[0]
        d_out = y_pred.shape[1]

        n_layers = len(self.W)
        delta = 2.0 * (y_pred - y_true) / (n * d_out)
        delta *= _act_backward(self.output_act, self._cache_z[-1], y_pred)

        grads = [None] * n_layers
        for i in range(n_layers - 1, -1, -1):
            a_prev = self._cache_a[i]
            dW = a_prev.T @ delta
            db = np.sum(delta, axis=0)
            grads[i] = (dW, db)
            if i > 0:
                delta = (delta @ self.W[i].T) * \
                    _act_backward(self.hidden_act, self._cache_z[i - 1], self._cache_a[i])
        return grads

    # ── 权值导出/加载 ─────────────────────────────────────
    def get_weights(self):
        return {'W': [w.copy() for w in self.W], 'b': [b.copy() for b in self.b]}

    def n_params(self):
        return int(sum(w.size for w in self.W) + sum(b.size for b in self.b))


class Adam:
    """Adam 优化器 (Kingma & Ba, 2015)"""

    def __init__(self, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        self.lr = lr
        self.b1 = beta1
        self.b2 = beta2
        self.eps = eps
        self.m = None
        self.v = None
        self.t = 0

    def step(self, params, grads):
        if self.m is None:
            self.m = [np.zeros_like(p) for p in params]
            self.v = [np.zeros_like(p) for p in params]
        self.t += 1
        bc1 = 1.0 - self.b1 ** self.t
        bc2 = 1.0 - self.b2 ** self.t
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1.0 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1.0 - self.b2) * g * g
            p -= self.lr * (self.m[i] / bc1) / (np.sqrt(self.v[i] / bc2) + self.eps)


def train_mlp(
    X, Y,
    sizes,
    hidden_act='tanh',
    output_act='tanh',
    epochs=60,
    batch_size=256,
    lr=1e-3,
    val_split=0.2,
    seed=0,
    verbose=True,
):
    """
    训练一个 NumpyMLP，返回 (model, history)。

    history: {'train_loss': [...], 'val_loss': [...], 'val_mse': [...]}
    """
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    n = X.shape[0]
    rng = np.random.default_rng(seed)
    idx = rng.permutation(n)
    n_val = max(1, int(n * val_split)) if val_split > 0 else 0
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    model = NumpyMLP(sizes, hidden_act=hidden_act, output_act=output_act, seed=seed)
    opt = Adam(lr=lr)
    history = {'train_loss': [], 'val_loss': []}

    X_tr, Y_tr = X[train_idx], Y[train_idx]
    X_va, Y_va = (X[val_idx], Y[val_idx]) if n_val else (None, None)

    for epoch in range(epochs):
        perm = rng.permutation(len(X_tr))
        losses = []
        for start in range(0, len(X_tr), batch_size):
            b = perm[start:start + batch_size]
            pred = model.forward(X_tr[b])
            loss = model.mse(pred, Y_tr[b])
            grads = model.backward(Y_tr[b])
            # 参数与梯度必须同序：所有 W 在前、所有 b 在后
            flat_params = model.W + model.b
            flat_grads = [g[0] for g in grads] + [g[1] for g in grads]
            opt.step(flat_params, flat_grads)
            losses.append(loss)
        train_loss = float(np.mean(losses))
        history['train_loss'].append(train_loss)

        if n_val:
            val_pred = model.forward(X_va)
            val_loss = model.mse(val_pred, Y_va)
            history['val_loss'].append(val_loss)
        else:
            val_loss = train_loss

        if verbose and (epoch % 10 == 0 or epoch == epochs - 1):
            print(f"  epoch {epoch + 1:3d}/{epochs} | train MSE={train_loss:.6f} | "
                  f"val MSE={val_loss:.6f}")

    return model, history


def save_npz(path, model, extra=None):
    """保存模型权重为 .npz（W0/b0/W1/b1/... 命名，跨版本稳定）"""
    payload = {}
    for i, w in enumerate(model.W):
        payload[f'w{i + 1}'] = w.astype(np.float32)
        payload[f'b{i + 1}'] = model.b[i].astype(np.float32)
    if extra:
        payload.update(extra)
    np.savez(path, **payload)


def load_npz(path):
    """加载 save_npz 保存的权重，返回 {'W': [...], 'b': [...]}"""
    data = np.load(path)
    W, b = [], []
    i = 1
    while f'w{i}' in data:
        W.append(data[f'w{i}'])
        b.append(data[f'b{i}'])
        i += 1
    return {'W': W, 'b': b}
