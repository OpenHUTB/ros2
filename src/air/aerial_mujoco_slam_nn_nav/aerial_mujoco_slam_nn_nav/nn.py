"""纯 NumPy 神经网络：MLP 与小卷积网络，含 Adam 训练器。

不依赖 PyTorch/TensorFlow——课程要求"神经网络"，但没要求框架；纯 NumPy 实现
便于在只有 ROS 基础环境的 Ubuntu 20.04 虚拟机上零依赖运行，也便于在报告里
逐行讲清前向/反向传播公式。
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# 参数初始化与基础算子
# ---------------------------------------------------------------------------
def init_mlp(sizes: List[int], rng: np.random.Generator, gain: float = 1.0) -> Dict[str, np.ndarray]:
    params = {}
    for i in range(len(sizes) - 1):
        fan_in, fan_out = sizes[i], sizes[i + 1]
        scale = gain * np.sqrt(2.0 / fan_in)
        params[f"W{i}"] = rng.normal(0.0, scale, (fan_in, fan_out)).astype(np.float32)
        params[f"b{i}"] = np.zeros(fan_out, dtype=np.float32)
    return params


def mlp_forward(params: Dict[str, np.ndarray], x: np.ndarray, cache: bool = False):
    """ReLU MLP 前向；最后一层线性。"""
    n_layers = len([k for k in params if k.startswith("W")])
    acts = [x]
    h = x
    for i in range(n_layers):
        z = h @ params[f"W{i}"] + params[f"b{i}"]
        if i < n_layers - 1:
            h = np.maximum(z, 0.0)
        else:
            h = z
        if cache:
            acts.append(h)
    return (h, acts) if cache else h


def mlp_backward(params: Dict[str, np.ndarray], acts: List[np.ndarray],
                 grad_out: np.ndarray) -> Dict[str, np.ndarray]:
    n_layers = len([k for k in params if k.startswith("W")])
    grads = {}
    g = grad_out
    for i in reversed(range(n_layers)):
        h_prev = acts[i]
        grads[f"W{i}"] = h_prev.T @ g
        grads[f"b{i}"] = g.sum(axis=0)
        if i > 0:
            g = (g @ params[f"W{i}"].T) * (acts[i] > 0).astype(np.float32)
    return grads


class Adam:
    """标准 Adam 优化器（自动跳过整型常量，如网络形状元数据）。"""

    def __init__(self, params: Dict[str, np.ndarray], lr: float = 1e-3,
                 beta1: float = 0.9, beta2: float = 0.999, eps: float = 1e-8,
                 weight_decay: float = 0.0):
        self.params = params
        self.trainable = [k for k, v in params.items()
                          if isinstance(v, np.ndarray) and np.issubdtype(v.dtype, np.floating)]
        self.lr = lr
        self.b1, self.b2, self.eps = beta1, beta2, eps
        self.wd = weight_decay
        self.m = {k: np.zeros_like(params[k]) for k in self.trainable}
        self.v = {k: np.zeros_like(params[k]) for k in self.trainable}
        self.t = 0

    def step(self, grads: Dict[str, np.ndarray]):
        self.t += 1
        for k in self.trainable:
            g = grads[k]
            if self.wd:
                g = g + self.wd * self.params[k]
            self.m[k] = self.b1 * self.m[k] + (1 - self.b1) * g
            self.v[k] = self.b2 * self.v[k] + (1 - self.b2) * (g * g)
            m_hat = self.m[k] / (1 - self.b1 ** self.t)
            v_hat = self.v[k] / (1 - self.b2 ** self.t)
            self.params[k] -= self.lr * m_hat / (np.sqrt(v_hat) + self.eps)


def mse_loss(pred: np.ndarray, target: np.ndarray) -> Tuple[float, np.ndarray]:
    diff = pred - target
    return float(np.mean(diff ** 2)), (2.0 / diff.size) * diff


def train_mlp(params, X, Y, epochs=40, batch=64, lr=1e-3, seed=0, l2=0.0,
              verbose=False, val=None) -> List[dict]:
    """小批量训练，返回逐 epoch 的 loss/val 记录。"""
    rng = np.random.default_rng(seed)
    opt = Adam(params, lr=lr, weight_decay=l2)
    n = len(X)
    hist = []
    for ep in range(epochs):
        idx = rng.permutation(n)
        tot = 0.0
        for i in range(0, n, batch):
            b = idx[i:i + batch]
            pred, acts = mlp_forward(params, X[b], cache=True)
            loss, g = mse_loss(pred, Y[b])
            grads = mlp_backward(params, acts, g.astype(np.float32))
            opt.step(grads)
            tot += loss * len(b)
        rec = {"epoch": ep, "loss": tot / n}
        if val is not None:
            pv = mlp_forward(params, val[0])
            rec["val_loss"] = float(np.mean((pv - val[1]) ** 2))
        hist.append(rec)
        if verbose and (ep % max(1, epochs // 5) == 0 or ep == epochs - 1):
            print(f"  epoch {ep:3d} loss={rec['loss']:.5f}" +
                  (f" val={rec['val_loss']:.5f}" if 'val_loss' in rec else ""))
    return hist


# ---------------------------------------------------------------------------
# 小型卷积网络（端到端视觉）：Conv(1->8,5x5,stride 2) + ReLU + FC
# ---------------------------------------------------------------------------
def init_cnn(h: int, w: int, n_filters: int = 8, k: int = 5, stride: int = 2,
             hidden: int = 32, out_dim: int = 2, rng=None) -> Dict[str, np.ndarray]:
    rng = rng or np.random.default_rng(0)
    ho = (h - k) // stride + 1
    wo = (w - k) // stride + 1
    return {
        "K": (rng.normal(0, np.sqrt(2.0 / (k * k)), (n_filters, k, k)).astype(np.float32)),
        "cb": np.zeros(n_filters, dtype=np.float32),
        "W1": (rng.normal(0, np.sqrt(2.0 / (n_filters * ho * wo)),
                          (n_filters * ho * wo, hidden)).astype(np.float32)),
        "b1": np.zeros(hidden, dtype=np.float32),
        "W2": (rng.normal(0, np.sqrt(2.0 / hidden), (hidden, out_dim)).astype(np.float32)),
        "b2": np.zeros(out_dim, dtype=np.float32),
        "shape": np.array([h, w, n_filters, k, stride, ho, wo], dtype=np.int32),
    }


def _im2col(x: np.ndarray, k: int, stride: int):
    n, c, h, w = x.shape
    ho = (h - k) // stride + 1
    wo = (w - k) // stride + 1
    cols = np.empty((n, c, k, k, ho, wo), dtype=np.float32)
    for i in range(k):
        for j in range(k):
            cols[:, :, i, j, :, :] = x[:, :, i:i + stride * ho:stride, j:j + stride * wo:stride]
    return cols.reshape(n, c * k * k, ho * wo), ho, wo


def cnn_forward(params, x, cache=False):
    """x: (N, H, W) 灰度深度图 -> (N, out_dim)。

    卷积层：``conv[n,f,p] = Σ_k cols[n,k,p] · K[f,k] + cb[f]``，
    其中 cols 是 im2col 展开的 (N, k*k, ho*wo)。
    """
    h, w, nf, k, stride, ho, wo = [int(v) for v in params["shape"]]
    x4 = x.reshape(-1, 1, h, w).astype(np.float32)
    cols, ho, wo = _im2col(x4, k, stride)
    conv = np.einsum("nkp,fk->nfp", cols, params["K"].reshape(nf, -1)) + params["cb"][None, :, None]
    a = np.maximum(conv, 0.0)
    feat = a.reshape(len(x), -1)
    z1 = feat @ params["W1"] + params["b1"]
    h1 = np.maximum(z1, 0.0)
    out = h1 @ params["W2"] + params["b2"]
    if cache:
        return out, (cols, conv, feat, z1, h1, ho, wo)
    return out


def cnn_backward(params, cache, grad_out):
    cols, conv, feat, z1, h1, ho, wo = cache
    h, w, nf, k, stride, _, _ = [int(v) for v in params["shape"]]
    grads = {}
    grads["W2"] = h1.T @ grad_out
    grads["b2"] = grad_out.sum(0)
    g1 = (grad_out @ params["W2"].T) * (z1 > 0)
    grads["W1"] = feat.T @ g1
    grads["b1"] = g1.sum(0)

    gconv = (g1 @ params["W1"].T).reshape(len(grad_out), nf, ho, wo)
    gconv = gconv * (conv.reshape(len(grad_out), nf, ho, wo) > 0)
    # dL/dK[f, i, j] = Σ_n Σ_p cols[n, i*k+j, p] * gconv[n, f, p]
    gk = np.einsum("nkp,nfp->fk", cols, gconv.reshape(len(grad_out), nf, ho * wo))
    grads["K"] = gk.reshape(nf, k, k).astype(np.float32)
    grads["cb"] = gconv.sum(axis=(0, 2, 3)).astype(np.float32)
    return grads


def train_cnn(params, X, Y, epochs=30, batch=32, lr=1e-3, seed=0, verbose=False):
    rng = np.random.default_rng(seed)
    opt = Adam(params, lr=lr)
    hist = []
    n = len(X)
    for ep in range(epochs):
        idx = rng.permutation(n)
        tot = 0.0
        for i in range(0, n, batch):
            b = idx[i:i + batch]
            out, cache = cnn_forward(params, X[b], cache=True)
            loss, g = mse_loss(out, Y[b])
            opt.step(cnn_backward(params, cache, g.astype(np.float32)))
            tot += loss * len(b)
        hist.append({"epoch": ep, "loss": tot / n})
        if verbose and (ep % max(1, epochs // 4) == 0 or ep == epochs - 1):
            print(f"  epoch {ep:3d} loss={hist[-1]['loss']:.5f}")
    return hist


# ---------------------------------------------------------------------------
def save_model(path: str, params: Dict[str, np.ndarray], meta: dict | None = None):
    np.savez_compressed(path, **{k: v for k, v in params.items() if isinstance(v, np.ndarray)},
                        **({f"meta_{k}": np.array(v) for k, v in (meta or {}).items()}))


def load_model(path: str) -> Dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files if not k.startswith("meta_")}
