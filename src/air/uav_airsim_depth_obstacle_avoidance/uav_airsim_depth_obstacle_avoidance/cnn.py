"""纯 NumPy 的小型 CNN，用于「深度图 -> 避障动作」回归。

结构（输入 1×64×64）::

    Conv(1->8, 5x5, stride 2) -> ReLU -> MaxPool(2) -> FC(8*15*15 -> 64) -> ReLU -> FC(64 -> 2) -> tanh

两个输出分别是 **偏航角速度** 与 **目标高度**，都归一化到 [-1, 1]。

不依赖 PyTorch / TensorFlow，便于在任何装了 numpy 的机器上复现；
梯度全部手写，前向/反向都在 ``forward`` / ``backward`` 里可对照检查。
"""
from __future__ import annotations

import numpy as np


def im2col(x, k, stride):
    """(C,H,W) -> (OH*OW, C*k*k)，按行展开每个感受野。"""
    c, h, w = x.shape
    oh = (h - k) // stride + 1
    ow = (w - k) // stride + 1
    cols = np.empty((oh * ow, c * k * k), dtype=np.float32)
    idx = 0
    for y in range(0, h - k + 1, stride):
        for xx in range(0, w - k + 1, stride):
            cols[idx] = x[:, y:y + k, xx:xx + k].reshape(-1)
            idx += 1
    return cols, oh, ow


def col2im(cols, x_shape, k, stride, oh, ow):
    """im2col 的逆过程，把梯度累加回输入形状。"""
    c, h, w = x_shape
    out = np.zeros((c, h, w), dtype=np.float32)
    idx = 0
    for y in range(0, h - k + 1, stride):
        for xx in range(0, w - k + 1, stride):
            out[:, y:y + k, xx:xx + k] += cols[idx].reshape(c, k, k)
            idx += 1
    return out


class Conv2D:
    def __init__(self, in_c, out_c, k=5, stride=2, rng=None):
        rng = rng or np.random
        scale = np.sqrt(2.0 / (in_c * k * k))
        self.W = (rng.randn(out_c, in_c, k, k) * scale).astype(np.float32)
        self.b = np.zeros(out_c, dtype=np.float32)
        self.k, self.stride = k, stride
        self.cache = None

    def forward(self, x):
        cols, oh, ow = im2col(x, self.k, self.stride)
        out = cols @ self.W.reshape(self.W.shape[0], -1).T + self.b
        out = out.T.reshape(self.W.shape[0], oh, ow)
        self.cache = (cols, x.shape, oh, ow)
        return out

    def backward(self, dout):
        cols, x_shape, oh, ow = self.cache
        d = dout.reshape(dout.shape[0], -1).T  # (OH*OW, out_c)
        self.dW = (d.T @ cols).reshape(self.W.shape)
        self.db = d.sum(axis=0).astype(np.float32)
        dcols = d @ self.W.reshape(self.W.shape[0], -1)
        dx = col2im(dcols, x_shape, self.k, self.stride, oh, ow)
        return dx


class ReLU:
    def forward(self, x):
        self.mask = x > 0
        return x * self.mask

    def backward(self, dout):
        return dout * self.mask


class MaxPool2D:
    def __init__(self, size=2):
        self.size = size

    def forward(self, x):
        c, h, w = x.shape
        s = self.size
        hh, ww = h // s, w // s
        xr = x[:, :hh * s, :ww * s].reshape(c, hh, s, ww, s)
        out = xr.max(axis=(2, 4))
        self.cache = (xr, x.shape)
        return out

    def backward(self, dout):
        xr, x_shape = self.cache
        s = self.size
        mask = xr == xr.max(axis=(2, 4), keepdims=True)
        dxr = mask * dout[:, :, None, :, None]
        c, h, w = x_shape
        dx = np.zeros((c, h, w), dtype=np.float32)
        hh, ww = h // s, w // s
        dx[:, :hh * s, :ww * s] = dxr.reshape(c, hh * s, ww * s)
        return dx


class Linear:
    def __init__(self, n_in, n_out, rng=None):
        rng = rng or np.random
        scale = np.sqrt(2.0 / n_in)
        self.W = (rng.randn(n_in, n_out) * scale).astype(np.float32)
        self.b = np.zeros(n_out, dtype=np.float32)

    def forward(self, x):
        self.x = x
        return x @ self.W + self.b

    def backward(self, dout):
        # 训练是逐样本进行的，x 与 dout 都是一维，所以这里用外积而不是矩阵乘
        self.dW = np.outer(self.x, dout).astype(np.float32)
        self.db = dout.astype(np.float32)
        return dout @ self.W.T


class DepthPolicyNet:
    """深度图 -> (yaw_rate_norm, altitude_norm)。"""

    def __init__(self, in_size=64, seed=0):
        rng = np.random.RandomState(seed)
        self.conv = Conv2D(1, 8, k=5, stride=2, rng=rng)
        self.relu1 = ReLU()
        self.pool = MaxPool2D(2)
        self.fc1 = Linear(8 * 15 * 15, 64, rng=rng)
        self.relu2 = ReLU()
        self.fc2 = Linear(64, 2, rng=rng)
        self.layers = [self.conv, self.relu1, self.pool, self.fc1, self.relu2, self.fc2]

    # ------------------------------------------------------------ 前向
    def forward(self, x):
        h = self.conv.forward(x[None])
        h = self.relu1.forward(h)
        h = self.pool.forward(h)
        h = h.reshape(-1)
        h = self.fc1.forward(h)
        h = self.relu2.forward(h)
        y = self.fc2.forward(h)
        return np.tanh(y)

    def predict(self, x):
        return self.forward(x.astype(np.float32))

    # ------------------------------------------------------------ 反向
    def backward(self, dout):
        """dout: (2,) 对 tanh 输出前的梯度（外部已乘 tanh'）。"""
        d = self.fc2.backward(dout)
        d = self.relu2.backward(d)
        d = self.fc1.backward(d)
        d = d.reshape(8, 15, 15)
        d = self.pool.backward(d)
        d = self.relu1.backward(d)
        return self.conv.backward(d)

    # ------------------------------------------------------------ 保存
    def params(self):
        return {
            "conv_W": self.conv.W, "conv_b": self.conv.b,
            "fc1_W": self.fc1.W, "fc1_b": self.fc1.b,
            "fc2_W": self.fc2.W, "fc2_b": self.fc2.b,
        }

    def load_state(self, state):
        self.conv.W = state["conv_W"]
        self.conv.b = state["conv_b"]
        self.fc1.W = state["fc1_W"]
        self.fc1.b = state["fc1_b"]
        self.fc2.W = state["fc2_W"]
        self.fc2.b = state["fc2_b"]

    def save(self, path):
        np.savez(path, **self.params())

    @classmethod
    def load(cls, path):
        net = cls()
        data = np.load(path)
        net.load_state(data)
        return net


class Adam:
    """最小 Adam 实现，只为训练上面这一个网络。"""

    def __init__(self, net, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        self.net = net
        self.lr, self.b1, self.b2, self.eps = lr, beta1, beta2, eps
        self.m = {}
        self.v = {}
        self.t = 0
        for name in ["conv", "fc1", "fc2"]:
            layer = getattr(net, name)
            self.m[name] = {"W": np.zeros_like(layer.W), "b": np.zeros_like(layer.b)}
            self.v[name] = {"W": np.zeros_like(layer.W), "b": np.zeros_like(layer.b)}

    def step(self):
        self.t += 1
        for name in ["conv", "fc1", "fc2"]:
            layer = getattr(self.net, name)
            for p in ["W", "b"]:
                g = getattr(layer, "d" + p)
                self.m[name][p] = self.b1 * self.m[name][p] + (1 - self.b1) * g
                self.v[name][p] = self.b2 * self.v[name][p] + (1 - self.b2) * (g * g)
                mhat = self.m[name][p] / (1 - self.b1 ** self.t)
                vhat = self.v[name][p] / (1 - self.b2 ** self.t)
                new = getattr(layer, p) - self.lr * mhat / (np.sqrt(vhat) + self.eps)
                setattr(layer, p, new.astype(np.float32))
