# -*- coding: utf-8 -*-
"""滑移补偿神经网络（纯 NumPy，手写反向传播）。

为什么用神经网络而不是继续加标定系数：
    MuJoCo 实测显示，纵向滑移随**转向率**单调上升（ω=0 时 5.3%，ω=1.2 时 17.2%），
    这是轮胎横向刮擦的非线性效应，用常数系数 c_s / c_theta 无论如何都拟合不了。
    MLP 的输入就是能反映这种工况的量：左右轮速、IMU 角速度、纵向加速度。

网络结构：
    输入 4 维 [ω_l, ω_r, ω_imu, a_x] → 隐层 24 → 隐层 24 → 输出 2 维 [k_s, k_θ]
    其中 k_s、k_θ 是**补偿系数**：
        ds_corrected  = k_s · ds_odom
        dθ_corrected  = k_θ · dθ_odom
    训练目标就是让补偿后的位移/转角等于真值。

为什么输出"系数"而不是"速度"：
    系数与绝对速度无关，泛化到没见过的速度更容易；而且当网络失效时
    k_s≈k_θ≈1，系统退化成原始里程计，不会发散 —— 这是工程上的安全兜底。
"""

import numpy as np


class StandardScaler(object):

    def __init__(self, mean=None, std=None):
        self.mean = mean
        self.std = std

    def fit(self, x):
        x = np.asarray(x, dtype=np.float64)
        self.mean = x.mean(axis=0)
        std = x.std(axis=0)
        self.std = np.where(std < 1e-8, 1.0, std)
        return self

    def transform(self, x):
        return (np.asarray(x, dtype=np.float64) - self.mean) / self.std

    def save(self, path):
        np.savez(path, mean=self.mean, std=self.std)

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        return cls(d["mean"], d["std"])


class SlipMLP(object):
    """4 → h1 → h2 → 2，tanh 激活，输出线性。"""

    def __init__(self, hidden=(24, 24), seed=0):
        sizes = [4, hidden[0], hidden[1], 2]
        rng = np.random.RandomState(seed)
        self.W = []
        self.b = []
        for i in range(len(sizes) - 1):
            scale = np.sqrt(1.0 / sizes[i])
            self.W.append(rng.randn(sizes[i], sizes[i + 1]) * scale)
            self.b.append(np.zeros(sizes[i + 1]))
        self.sizes = sizes

    # ------------------------------------------------------------------
    def forward(self, x, cache=False):
        a = np.asarray(x, dtype=np.float64)
        acts = [a]
        for i in range(len(self.W) - 1):
            a = np.tanh(a.dot(self.W[i]) + self.b[i])
            acts.append(a)
        out = a.dot(self.W[-1]) + self.b[-1]
        if cache:
            return out, acts
        return out

    def predict(self, x):
        return self.forward(x)

    def backward(self, x, y, lr=1e-3, l2=1e-6):
        out, acts = self.forward(x, cache=True)
        y = np.asarray(y, dtype=np.float64)
        diff = out - y
        loss = float(np.mean(diff ** 2))
        delta = 2.0 * diff / diff.size
        for i in range(len(self.W) - 1, -1, -1):
            gW = acts[i].T.dot(delta) + l2 * self.W[i]
            gb = delta.sum(axis=0)
            if i > 0:
                delta = delta.dot(self.W[i].T) * (1.0 - acts[i] ** 2)
            np.clip(gW, -1.0, 1.0, out=gW)
            np.clip(gb, -1.0, 1.0, out=gb)
            self.W[i] -= lr * gW
            self.b[i] -= lr * gb
        return loss

    # ------------------------------------------------------------------
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
        sizes = [d["W0"].shape[0]] + [int(d["W%d" % i].shape[1]) for i in range(n)]
        net = cls(hidden=(sizes[1], sizes[2]))
        net.W = [d["W%d" % i].astype(np.float64) for i in range(n)]
        net.b = [d["b%d" % i].astype(np.float64) for i in range(n)]
        return net


def features(wl, wr, gyro_z, acc_x):
    """把一帧传感器读数包成网络输入。"""
    return np.array([wl, wr, gyro_z, acc_x], dtype=np.float64)


def compensate(net, scaler, wl, wr, gyro_z, acc_x, ds_odom, dth_odom):
    """用网络输出的系数修正一个控制周期的位移与转角。"""
    k = net.predict(scaler.transform(features(wl, wr, gyro_z, acc_x).reshape(1, -1)))[0]
    # 兜底：网络输出异常时退回原始里程计，避免发散
    k_s = float(np.clip(k[0], 0.5, 1.5))
    k_th = float(np.clip(k[1], 0.5, 1.5))
    return ds_odom * k_s, dth_odom * k_th
