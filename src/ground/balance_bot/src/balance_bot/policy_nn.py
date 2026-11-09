# -*- coding: utf-8 -*-
"""平衡策略网络（纯 NumPy，手写反向传播）。

为什么用行为克隆而不是强化学习：
    PD 控制器在仿真里已经能稳稳站住（20 s 最大倾角 0.96°），
    那就直接把它当"老师"，采集 (状态, 输出力矩) 让网络去学。
    有现成的 teacher，训练是监督学习，几秒收敛；
    强化学习则要试几百个 episode、还容易不收敛 —— 对课程作业没必要。

输入 4 维  [θ, θ̇, v, v_target]  → 隐层 16 → 16 → 输出 1 维 τ（左右轮同值）
"""

import numpy as np


class StandardScaler(object):

    def __init__(self, mean=None, std=None):
        self.mean = mean
        self.std = std

    def fit(self, x):
        x = np.asarray(x, dtype=np.float64)
        self.mean = x.mean(axis=0)
        s = x.std(axis=0)
        self.std = np.where(s < 1e-8, 1.0, s)
        return self

    def transform(self, x):
        return (np.asarray(x, dtype=np.float64) - self.mean) / self.std

    def save(self, path):
        np.savez(path, mean=self.mean, std=self.std)

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        return cls(d["mean"], d["std"])


class PolicyMLP(object):
    """4 → 16 → 16 → 1，tanh 激活，输出线性。"""

    def __init__(self, hidden=(16, 16), seed=0):
        sizes = [4, hidden[0], hidden[1], 1]
        rng = np.random.RandomState(seed)
        self.W = [rng.randn(sizes[i], sizes[i + 1]) * np.sqrt(1.0 / sizes[i])
                  for i in range(len(sizes) - 1)]
        self.b = [np.zeros(sizes[i + 1]) for i in range(len(sizes) - 1)]
        self.sizes = sizes

    def forward(self, x, cache=False):
        a = np.asarray(x, dtype=np.float64)
        h = [a]
        for i in range(len(self.W) - 1):
            a = np.tanh(a.dot(self.W[i]) + self.b[i])
            h.append(a)
        out = a.dot(self.W[-1]) + self.b[-1]
        return (out, h) if cache else out

    def predict(self, x):
        return self.forward(x)

    def backward(self, x, y, lr=1e-3, l2=1e-6):
        out, h = self.forward(x, cache=True)
        diff = out - np.asarray(y, dtype=np.float64)
        loss = float(np.mean(diff ** 2))
        delta = 2.0 * diff / diff.size
        for i in range(len(self.W) - 1, -1, -1):
            gW = h[i].T.dot(delta) + l2 * self.W[i]
            gb = delta.sum(axis=0)
            if i > 0:
                delta = delta.dot(self.W[i].T) * (1.0 - h[i] ** 2)
            # 裁剪梯度：早期没裁剪时，个别大误差样本会把整批权重打飞
            np.clip(gW, -1.0, 1.0, out=gW)
            np.clip(gb, -1.0, 1.0, out=gb)
            self.W[i] -= lr * gW
            self.b[i] -= lr * gb
        return loss

    def save(self, path):
        payload = {"n": len(self.W)}
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            payload["W%d" % i] = w
            payload["b%d" % i] = b
        np.savez(path, **payload)

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        n = int(d["n"])
        net = cls.__new__(cls)
        net.W = [d["W%d" % i].astype(np.float64) for i in range(n)]
        net.b = [d["b%d" % i].astype(np.float64) for i in range(n)]
        net.sizes = [net.W[0].shape[0]] + [w.shape[1] for w in net.W]
        return net


def state_features(st, v_target):
    """把一个状态包成网络输入。刻意只包含**传感器能测到**的量。"""
    return np.array([st.theta, st.theta_dot, st.v, v_target], dtype=np.float64)
