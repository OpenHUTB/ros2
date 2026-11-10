"""nn.py —— 纯 NumPy 时序 MLP（3 帧 768 维 → 15 维关节目标角）

相对原实现的三处改动：

1. **输入 / 输出标准化**：768 维池化图像格子里有大量近似恒定的像素（背景、
   地面），不标准化时它们的大尺度会压低有效特征的梯度；15 维输出的量纲也不一致
   （踝 ±0.2 rad、肩肘 -1.4 rad），不标准化会让 MSE 被大关节主导。
2. **std 下限 0.01**：近似恒定的输入维标准化后会被放大成噪声，
   实测不设下限时，只在 pitch 差 1° 的站立输入上预测就会从 0.00 翻到 1.00。
3. **Adam 替代一阶动量**：原 ``m = 0.9m + 0.1g``（无二阶矩、lr 固定）在
   类别/量纲不平衡时收敛极慢，且容易停在“输出均值”这个局部解上。
"""
from __future__ import annotations

import numpy as np

STD_FLOOR = 0.01


class TemporalMLP:
    def __init__(self, input_dim=768, hidden1=256, hidden2=128, output_dim=15, seed=0):
        rng = np.random.default_rng(seed)
        self.input_dim, self.output_dim = input_dim, output_dim
        self.W1 = rng.normal(0, np.sqrt(2.0 / input_dim), (input_dim, hidden1))
        self.b1 = np.zeros(hidden1)
        self.W2 = rng.normal(0, np.sqrt(2.0 / hidden1), (hidden1, hidden2))
        self.b2 = np.zeros(hidden2)
        self.W3 = rng.normal(0, np.sqrt(1.0 / hidden2), (hidden2, output_dim))
        self.b3 = np.zeros(output_dim)

        # 标准化参数（训练时由 fit_normalizer 写入，随权重一起保存）
        self.x_mean = np.zeros(input_dim)
        self.x_std = np.ones(input_dim)
        self.y_mean = np.zeros(output_dim)
        self.y_std = np.ones(output_dim)

        self._adam_m = [np.zeros_like(p) for p in self._params()]
        self._adam_v = [np.zeros_like(p) for p in self._params()]
        self._adam_t = 0

    # ---------- 标准化 ----------
    def fit_normalizer(self, X, y):
        self.x_mean = X.mean(axis=0)
        self.x_std = np.maximum(X.std(axis=0), STD_FLOOR)
        self.y_mean = y.mean(axis=0)
        self.y_std = np.maximum(y.std(axis=0), STD_FLOOR)
        return self

    def _norm_x(self, x):
        return (x - self.x_mean) / self.x_std

    def _denorm_y(self, yn):
        return yn * self.y_std + self.y_mean

    # ---------- 前向 / 反向 ----------
    def _params(self):
        return [self.W1, self.b1, self.W2, self.b2, self.W3, self.b3]

    def _forward_norm(self, xn):
        self.z1 = xn @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        return self.h2 @ self.W3 + self.b3

    def forward(self, x):
        """输入原始特征，输出原始量纲的 15 维关节角。"""
        return self._denorm_y(self._forward_norm(self._norm_x(x)))

    def predict(self, x):
        return self.forward(x)

    def train_step(self, x, y, lr=1e-3):
        """一步 Adam。x/y 为原始量纲，内部标准化；返回标准化空间的 MSE。"""
        xn = self._norm_x(x)
        yn = (y - self.y_mean) / self.y_std
        pred = self._forward_norm(xn)
        n = x.shape[0]
        d_out = 2 * (pred - yn) / n
        d_W3 = self.h2.T @ d_out
        d_b3 = d_out.sum(0)
        d_h2 = d_out @ self.W3.T
        d_z2 = d_h2 * (self.z2 > 0)
        d_W2 = self.h1.T @ d_z2
        d_b2 = d_z2.sum(0)
        d_h1 = d_z2 @ self.W2.T
        d_z1 = d_h1 * (self.z1 > 0)
        d_W1 = xn.T @ d_z1
        d_b1 = d_z1.sum(0)

        grads = [d_W1, d_b1, d_W2, d_b2, d_W3, d_b3]
        self._adam_t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for i, (p, g) in enumerate(zip(self._params(), grads)):
            self._adam_m[i] = b1 * self._adam_m[i] + (1 - b1) * g
            self._adam_v[i] = b2 * self._adam_v[i] + (1 - b2) * (g * g)
            mh = self._adam_m[i] / (1 - b1 ** self._adam_t)
            vh = self._adam_v[i] / (1 - b2 ** self._adam_t)
            p -= lr * mh / (np.sqrt(vh) + eps)
        return float(np.mean((pred - yn) ** 2))

    # ---------- 存取 ----------
    def save(self, path):
        np.savez(path,
                 W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2, W3=self.W3, b3=self.b3,
                 x_mean=self.x_mean, x_std=self.x_std, y_mean=self.y_mean, y_std=self.y_std)

    def load(self, path):
        d = np.load(path)
        self.W1, self.b1 = d["W1"], d["b1"]
        self.W2, self.b2 = d["W2"], d["b2"]
        self.W3, self.b3 = d["W3"], d["b3"]
        if "x_mean" in d.files:
            self.x_mean, self.x_std = d["x_mean"], d["x_std"]
            self.y_mean, self.y_std = d["y_mean"], d["y_std"]
        else:
            # 旧格式权重（没有标准化参数）：按恒等标准化加载，行为与原实现一致；
            # 推理结果不可信，需要用 --train 重新生成带标准化的权重。
            self.x_mean = np.zeros(self.input_dim)
            self.x_std = np.ones(self.input_dim)
            self.y_mean = np.zeros(self.output_dim)
            self.y_std = np.ones(self.output_dim)
        return self
