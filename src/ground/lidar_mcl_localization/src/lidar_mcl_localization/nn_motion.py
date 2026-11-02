# -*- coding: utf-8 -*-
"""神经网络运动模型修正（4 → 24 → 24 → 3 的 MLP，纯 NumPy 手写反向传播）。

为什么需要一个神经网络
----------------------
里程计增量模型的四个 α 是**手调的常数**，但真实打滑是工况的函数：
低速小转角时几乎不打滑（α 应该很小），急转弯时滑移率能到 10% 以上（α 应该大）。
常数 α 只能取折中：调小 → 急转弯跟不上、定位发散；调大 → 直行时粒子乱飞、抖动。

本模块用一个小 MLP 从**最近一段里程计统计量**预测三件事：

    k_s        路程修正系数      真实位移 ≈ k_s · 里程计位移
    k_th       转角修正系数      真实转角 ≈ k_th · 里程计转角
    noise      噪声标度          等比放大/缩小 α₁~α₄（打滑大 → 散开；平稳 → 收紧）

输入 4 维（都是最近 N 帧里程计的统计量，不需要真值）：
    [ds, dth, |dth|/max(ds,eps), 该段内 |dth| 的累计量]

输出 3 维，经过有界映射，保证网络失效时退化为原始里程计（安全兜底）：
    k_s  ∈ [0.8, 1.2]，k_th ∈ [0.8, 1.2]，noise ∈ [0.25, 4.0]
"""

import math

import numpy as np

FEATURE_DIM = 4
OUTPUT_DIM = 3


class StandardScaler(object):
    """逐维标准化。标准差为 0 的维度退化为 1，避免除零。"""

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
        if self.mean is None:
            return np.asarray(x, dtype=np.float64)
        return (np.asarray(x, dtype=np.float64) - self.mean) / self.std

    def to_dict(self):
        return dict(mean=self.mean, std=self.std)

    @classmethod
    def from_dict(cls, d):
        return cls(np.asarray(d["mean"]), np.asarray(d["std"]))


class MotionMLP(object):
    """3 层 MLP：4 → 24 → 24 → 3，tanh 隐层、线性输出。"""

    def __init__(self, hidden=(24, 24), seed=0):
        sizes = [FEATURE_DIM, hidden[0], hidden[1], OUTPUT_DIM]
        rng = np.random.RandomState(seed)
        self.W = []
        self.b = []
        for i in range(len(sizes) - 1):
            scale = math.sqrt(1.0 / sizes[i])
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
        # 梯度按**样本数**归一，不是按 diff.size（=批量×输出维）。
        # 按 diff.size 归一会让梯度小一个输出维的倍数（这里 3 倍），
        # 再叠加小批量（几十条）就会小两个数量级——表现为"训练几百轮 loss 几乎不动"。
        delta = 2.0 * diff / max(diff.shape[0], 1)
        for i in range(len(self.W) - 1, -1, -1):
            gW = acts[i].T.dot(delta) + l2 * self.W[i]
            gb = delta.sum(axis=0)
            if i > 0:
                delta = delta.dot(self.W[i].T) * (1.0 - acts[i] ** 2)
            self.W[i] -= lr * gW
            self.b[i] -= lr * gb
        return loss

    # ------------------------------------------------------------------
    def to_dict(self):
        payload = {"n": len(self.W)}
        for i, (w, b) in enumerate(zip(self.W, self.b)):
            payload["W%d" % i] = w
            payload["b%d" % i] = b
        return payload

    def save(self, path):
        np.savez(path, **self.to_dict())

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        n = int(d["n"])
        net = cls(hidden=(int(d["W0"].shape[1]), int(d["W1"].shape[1])))
        net.W = [d["W%d" % i].astype(np.float64) for i in range(n)]
        net.b = [d["b%d" % i].astype(np.float64) for i in range(n)]
        return net


def features_from_delta(ds, dth, dt=0.1):
    """把一次里程计增量包成网络输入（4 维，全部有界）。

    特征设计踩过两个坑，这里记录一下，免得后来人重犯：

    1. **不要用比值**：``|dθ|/ds`` 在纯旋转（ds≈0）时会炸到十几，
       动态范围太大，小样本下网络会在直行工况外推出 k_s=1.145（训练标签最大 0.96）。
    2. **不要用累加量**：训练时按 20 步累加 ``Σ|dθ|``、推理时按 10 步窗口，
       同一个工况的特征值差一倍，网络看到的是"没见过的输入"，直接外推。

    所以四个特征都取**与窗口长度无关、且有界**的量：

        v = ds/dt         线速度（m/s），纵向滑移的主要自变量
        ω = dθ/dt         角速度（rad/s），转向滑移的自变量
        |ω|               显式给一个绝对值通道，便于学"左右对称"的规律
        v·|ω|             速度与转向率的耦合项，横向刮擦与二者都有关
    """
    v = ds / max(dt, 1e-9)
    w = dth / max(dt, 1e-9)
    return np.array([v, w, abs(w), v * abs(w)], dtype=np.float64)


def correction_from_output(raw):
    """把网络输出映射成 (k_s, k_th, noise)，并做安全截断。

    这里**不能**再套一层 tanh 压缩：训练目标本身就是 k（例如 0.91），
    如果推理时再算 ``1 + 0.2·tanh(k)``，0.91 会被映射成 1.14 ——
    训练与推理的映射不一致，网络学得再准，输出也一定是错的。
    （这个坑我踩过一次：修正后里程计误差反而从 0.345 m 涨到 0.403 m。）

    安全性靠**截断**保证而不靠 tanh：即使网络完全失效，
    k 也只会在 [0.8, 1.2] 内，最多 20% 的偏差，不会发散。
    """
    raw = np.asarray(raw, dtype=np.float64).reshape(-1)
    k_s = float(np.clip(raw[0], 0.80, 1.20))
    k_th = float(np.clip(raw[1], 0.80, 1.20))
    noise = float(np.clip(raw[2], 0.50, 4.00))
    return k_s, k_th, noise


class NNMotionCorrector(object):
    """把网络、标准化器与特征缓存打包成一个可调用对象，丢给 MotionModel 使用。"""

    def __init__(self, net, scaler, window=10, dt=0.1):
        self.net = net
        self.scaler = scaler
        self.dt = float(dt)
        self.window = int(window)
        self.history = []
        self.enabled = net is not None

    def push(self, ds, dth):
        self.history.append((float(ds), float(dth)))
        if len(self.history) > self.window:
            self.history.pop(0)

    def features(self):
        """窗口内的平均速度/角速度 → 特征。

        用**平均**而不是累加：窗口长度变了（训练 20 步、在线 10 步）也不会让特征漂移。
        """
        if not self.history:
            return features_from_delta(0.0, 0.0, self.dt)
        ds = sum(h[0] for h in self.history) / len(self.history)
        dth = sum(h[1] for h in self.history) / len(self.history)
        return features_from_delta(ds, dth, self.dt)

    def __call__(self, context=None):
        """MotionModel 调用：返回 (ds_fixed, dth_fixed, noise_scale)。"""
        delta = context if context is not None else (0.0, 0.0)
        ds, dth = float(delta[0]), float(delta[1])
        if not self.enabled:
            return ds, dth, 1.0

        feat = self.features()
        raw = self.net.predict(self.scaler.transform(feat.reshape(1, -1)))[0]
        k_s, k_th, noise = correction_from_output(raw)
        return ds * k_s, dth * k_th, noise
