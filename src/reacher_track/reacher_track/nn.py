# -*- coding: utf-8 -*-
"""
nn.py
纯 NumPy 实现的多层感知机(MLP) + Adam 优化器。
用于监督学习: 由感知状态(state) 映射到 机械臂关节扭矩(ctrl)。
激活函数用 tanh(输出层线性), 损失用 MSE。
"""
import numpy as np


def xavier_init(fan_in, fan_out):
    """Xavier 初始化。"""
    return np.random.randn(fan_in, fan_out) * np.sqrt(1.0 / fan_in)


class MLP:
    """全连接网络, 结构如 [6, 64, 64, 2]。"""

    def __init__(self, dims, seed=None):
        if seed is not None:
            np.random.seed(seed)
        self.dims = list(dims)
        self.W = []
        self.b = []
        for i in range(len(self.dims) - 1):
            self.W.append(xavier_init(self.dims[i], self.dims[i + 1]))
            self.b.append(np.zeros(self.dims[i + 1]))
        self.n_layer = len(self.W)

    def forward(self, x):
        """前向: 返回输出层结果, 缓存中间量用于反向。x: (N, in) 或 (in,)"""
        self.a = [np.asarray(x, dtype=np.float64)]
        self.z = []
        h = self.a[0]
        for i in range(self.n_layer - 1):
            z = h @ self.W[i] + self.b[i]
            self.z.append(z)
            h = np.tanh(z)
            self.a.append(h)
        z = h @ self.W[-1] + self.b[-1]
        self.z.append(z)
        self.a.append(z)   # 输出层激活(线性) 存到 a[-1]
        return z

    def backward(self, grad_out):
        """反向传播: grad_out 为对输出层的梯度 (N,out)。
        返回 (grad_W_list, grad_b_list), 与 W/b 顺序一致。"""
        g = grad_out
        grads_w = [None] * self.n_layer
        grads_b = [None] * self.n_layer
        for i in range(self.n_layer - 1, -1, -1):
            n = g.shape[0]
            grads_w[i] = self.a[i].T @ g / n
            grads_b[i] = g.mean(axis=0)
            if i > 0:
                # 进入 tanh 层之前: d(tanh)/dz = 1 - tanh^2 = 1 - a[i]^2
                g = (g @ self.W[i].T) * (1.0 - self.a[i] ** 2)
        return grads_w, grads_b

    def mse_grad(self, y_true):
        """均方误差对输出层的梯度: (a[-1]-y)/N。"""
        return self.a[-1] - y_true

    def params(self):
        return self.W, self.b

    def save(self, path):
        d = {f"W{i}": w for i, w in enumerate(self.W)}
        d.update({f"b{i}": b for i, b in enumerate(self.b)})
        d["dims"] = np.array(self.dims)
        np.savez(path, **d)

    def load(self, path):
        d = np.load(path)
        self.dims = list(d["dims"])
        self.n_layer = len(self.dims) - 1
        self.W = [d[f"W{i}"] for i in range(self.n_layer)]
        self.b = [d[f"b{i}"] for i in range(self.n_layer)]


class Adam:
    """Adam 优化器。"""

    def __init__(self, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        self.lr = lr
        self.b1 = beta1
        self.b2 = beta2
        self.eps = eps
        self.m = []
        self.v = []
        self.t = 0
        self._init_done = False

    def init_mv(self, W, b):
        self.m = [np.zeros_like(w) for w in W] + [np.zeros_like(x) for x in b]
        self.v = [np.zeros_like(w) for w in W] + [np.zeros_like(x) for x in b]
        self._init_done = True

    def step(self, W, b, grads_w, grads_b):
        if not self._init_done:
            self.init_mv(W, b)
        self.t += 1
        params = W + b
        grads = grads_w + grads_b
        new_params = []
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * (g * g)
            m_hat = self.m[i] / (1 - self.b1 ** self.t)
            v_hat = self.v[i] / (1 - self.b2 ** self.t)
            new_params.append(p - self.lr * m_hat / (np.sqrt(v_hat) + self.eps))
        nw = len(W)
        for i in range(nw):
            W[i] = new_params[i]
            b[i] = new_params[nw + i]
