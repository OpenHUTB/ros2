#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 无人车作业 · 统一最小神经网络库（纯 numpy，可离线训练/推理）。

为满足老师"感知、规划、控制、端到端算法需要为神经网络"的硬性要求，本库提供
四个可训练/推理的**轻量神经网络**（只依赖 numpy，不依赖 TensorFlow/GPU）：

  - MLPClassifier      感知器：由特征向量 → 类别（如"有/无障碍"、"目标偏左/右"）
  - MLPPolicy          控制/规划策略网络：由状态向量 → 连续控制量（steer / 前进量）
  - build_end_to_end_cnn   (可选，在 04_end_to_end 用 tf.keras 的 CNN，见该模块)

统一接口：

    # 离线训练（不连 CARLA，可在本机 n0umpy 环境运行）：
    net = MLPClassifier([4, 8, 2])          # 输入4维 → 隐藏8 → 二类
    net.train(X, Y, epochs=200, lr=0.1)
    net.save("nn_model.json")

    # 在线推理（连 CARLA 时调用）：
    net = MLPClassifier.load("nn_model.json")
    prob = net.predict_proba(np.array(x))[0]   # 各类概率
    y    = int(net.predict(np.array(x))[0])    # 类别

所有数值计算用 numpy，含：前向传播、交叉熵/MSE 损失、反向传播、softmax/relu/tanh。

说明：为便于课程讲解，本库刻意保持"小而清晰"，适合作为神经网络入门示例；
实际生产可用 TensorFlow/PyTorch。文档对应公式见 docs/02、docs/03。
"""

import json
import os

import numpy as np


# ----------------------------------------------------------------------------
# 激活函数及其导数 / softmax
# ----------------------------------------------------------------------------
def relu(z):
    return np.maximum(0.0, z)


def relu_grad(a):
    """relu 导数（传入经过 relu 的激活值 a）。"""
    return np.where(a > 0, 1.0, 0.0)


def tanh(z):
    return np.tanh(z)


def softmax(z):
    e = np.exp(z - z.max(axis=1, keepdims=True))  # 数值稳定
    return e / e.sum(axis=1, keepdims=True)


# ----------------------------------------------------------------------------
# 基础 MLP：全连接 + relu 隐藏层 + softmax 输出（分类） / 线性或 tanh 输出（回归）
# ----------------------------------------------------------------------------
class _MLPBase:
    """共享的正向/反向/训练/存取实现；子类只决定输出层与损失。"""

    def __init__(self, layers, seed=0, activation="relu", weight_scale=0.3):
        """layers: 如 [4, 8, 2]，含输入维、各隐藏层、输出维。"""
        self.layers = layers
        self.activation = activation
        self.rng = np.random.RandomState(seed)
        self.W = []   # 每层权重
        self.b = []   # 每层偏置
        for i in range(len(layers) - 1):
            # He 初始化保证梯度稳定
            fan_in = layers[i]
            std = np.sqrt(2.0 / fan_in) * weight_scale
            self.W.append(self.rng.randn(layers[i + 1], layers[i]) * std)
            self.b.append(np.zeros(layers[i + 1]))

    # ---- 前向 ----
    def forward(self, X):
        """X:(N,D). 返回 (activations) 其中 activations[-1] 为输出层前各隐藏激活。"""
        a = X
        acts = [a]
        for W, b in zip(self.W, self.b):
            z = a @ W.T + b
            a = relu(z)
            acts.append(a)
        # acts[-1] 是最后一个隐藏/输出前的 relu 激活；输出层在本方法外算
        return acts

    def _all_forward(self, X):
        """返回 (acts, logits)：acts 为每层 relu 激活，logits 为原始输出。做完整前向。"""
        a = X
        acts = [a]
        logits = None
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W.T + b
            if i < len(self.W) - 1:
                a = relu(z)
                acts.append(a)
            else:
                logits = z
        return acts, logits

    def _backward(self, X, d_logits):
        """给定输出 logits 的梯度 d_logits:(N,C) 做反向传播，返回 (dW,db)。"""
        acts, _ = self._all_forward(X)
        dW = []
        db = []
        d = d_logits
        for i in reversed(range(len(self.W))):
            a = acts[i]          # 本层输入
            dW.insert(0, d.T @ a)
            db.insert(0, d.sum(axis=0))
            if i > 0:
                W = self.W[i]
                da = d @ W        # 传到本层输入
                # 上一激活是 relu(a)，其梯度要乘 relu 导数
                d = da * relu_grad(a)
        return dW, db

    def train(self, X, Y, epochs=200, lr=0.1, batch=64, verbose=10,
              X_val=None, Y_val=None):
        """随机批量梯度下降。

        X:(N,D)，Y 形状由子类决定（分类:类标，回归:连续值）。返回 history(dict)。
        """
        X = np.asarray(X, dtype=np.float32)
        Y = np.asarray(Y, dtype=np.float32)
        n = len(X)
        hist = {"loss": [], "acc": []}
        for ep in range(1, epochs + 1):
            idx = self.rng.permutation(n)
            loss_sum, acc_sum, cnt = 0.0, 0.0, 0
            for s in range(0, n, batch):
                b = idx[s:s + batch]
                Xb, Yb = X[b], Y[b]
                loss, d_logits, acc_c = self._step_loss(Xb, Yb)
                dW, db = self._backward(Xb, d_logits)
                for i in range(len(self.W)):
                    self.W[i] -= lr * dW[i]
                    self.b[i] -= lr * db[i]
                loss_sum += loss * len(b)
                acc_sum += acc_c * len(b)
                cnt += len(b)
            hist["loss"].append(loss_sum / cnt)
            hist["acc"].append(acc_sum / cnt)
            if verbose and ep % verbose == 0:
                print(f"  epoch {ep:4d} loss={hist['loss'][-1]:.4f} "
                      f"train_acc={hist['acc'][-1]:.3f}")
        return hist

    def _step_loss(self, Xb, Yb):
        """由子类实现，返回 (loss标量, d_logits, acc标量)。"""
        raise NotImplementedError

    def predict_proba(self, X):
        raise NotImplementedError

    def predict(self, X):
        raise NotImplementedError

    # ---- 序列化 ----
    def save(self, path):
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w") as f:
            json.dump({
                "type": self.__class__.__name__,
                "layers": self.layers,
                "W": [w.tolist() for w in self.W],
                "b": [t.tolist() for t in self.b],
            }, f)
        return path

    @classmethod
    def _load_state(cls, path):
        with open(path) as f:
            obj = json.load(f)
        return obj

    @classmethod
    def _from_state(cls, obj):
        net = cls(obj["layers"])
        net.W = [np.asarray(w, dtype=np.float32) for w in obj["W"]]
        net.b = [np.asarray(t, dtype=np.float32) for t in obj["b"]]
        return net


# ----------------------------------------------------------------------------
# 感知器：多分类 softmax + 交叉熵（用于障碍/偏航感知）
# ----------------------------------------------------------------------------
class MLPClassifier(_MLPBase):
    """softmax 多分类（交叉熵损失）。

    示例（传感器特征 → 三类）：
        net = MLPClassifier([5, 12, 3])      # 特征5维 → 隐藏12 → 3类
        Y = np.array([0,1,2, ...])           # 类别号
        net.train(X, Y, epochs=300)
    """

    def _step_loss(self, Xb, Yb):
        _, logits = self._all_forward(Xb)
        p = softmax(logits)
        y = Yb.astype(int)
        n = len(y)
        # 交叉熵损失
        loss = -np.mean(np.log(p[np.arange(n), y] + 1e-9))
        # d_logits：softmax 输出减去 one-hot
        d_logits = p.copy()
        d_logits[np.arange(n), y] -= 1.0
        d_logits /= n
        acc = float(np.mean(self.predict(Xb) == y))
        return loss, d_logits, acc

    def predict_proba(self, X):
        X = np.asarray(X, dtype=np.float32)
        if X.ndim == 1:
            X = X[None, :]
        _, logits = self._all_forward(X)
        return softmax(logits)

    def predict(self, X):
        return np.argmax(self.predict_proba(X), axis=1)

    def save(self, path):
        super().save(path)

    @classmethod
    def load(cls, path):
        return cls._from_state(cls._load_state(path))


# ----------------------------------------------------------------------------
# 策略/回归网络：relu 隐藏 + tanh 输出（用于控制/规划连续量）
# ----------------------------------------------------------------------------
class MLPPolicy(_MLPBase):
    """回归策略网络：状态 → 连续控制量（最后输出层用 tanh 或 linear）。

    示例（状态2维 → steer∈[-1,1] 回归）：
        net = MLPPolicy([2, 16, 1])
        Y = np.array([[0.3],[-0.5], ...])     # 连续标签
        net.train(X, Y, epochs=300)

    也支持多输出（如 [前进量, 转向]）→ 例 layers=[状态, 隐藏, 2]。
    """

    def __init__(self, layers, seed=0, activation="relu", output_act="tanh",
                 weight_scale=0.3):
        super().__init__(layers, seed=seed, activation=activation,
                         weight_scale=weight_scale)
        self.output_act = output_act  # 'linear' 或 'tanh'

    def _all_forward_out(self, X):
        a = X
        acts = [a]
        logits = None
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            z = a @ W.T + b
            if i < len(self.W) - 1:
                a = relu(z)
                acts.append(a)
            else:
                logits = z
        if self.output_act == "tanh":
            out = np.tanh(logits)
        else:
            out = logits
        return acts, logits, out

    def _step_loss(self, Xb, Yb):
        acts, logits, out = self._all_forward_out(Xb)
        # MSE
        diff = out - Yb
        n = len(Yb)
        loss = float(np.mean(np.square(diff)))
        # d_logits：MSE 对输出求导，再乘输出激活导数
        d_out = 2.0 * diff / n
        if self.output_act == "tanh":
            d_logits = d_out * (1.0 - out ** 2)
        else:
            d_logits = d_out
        acc = 0.0  # 回归不关心"准确率"
        return loss, d_logits, acc

    def predict(self, X):
        X = np.asarray(X, dtype=np.float32)
        if X.ndim == 1:
            X = X[None, :]
        _, _, out = self._all_forward_out(X)
        return out

    def save(self, path):
        super().save(path)

    @classmethod
    def load(cls, path):
        obj = cls._load_state(path)
        net = cls(obj["layers"])
        net.W = [np.asarray(w, dtype=np.float32) for w in obj["W"]]
        net.b = [np.asarray(t, dtype=np.float32) for t in obj["b"]]
        if "output_act" in obj:
            net.output_act = obj["output_act"]
        return net


# ----------------------------------------------------------------------------
# 简易 CNN（纯 numpy，图像 -> 连续量）——作业四端到端（图像->转向）也可离线验证
# ----------------------------------------------------------------------------
class SimpleCNN:
    """纯 numpy 实现的轻量 CNN，用于"图像 → 控制"的端到端演示与离线验证。

    结构：conv3x3(->relu,->2x2 maxpool) x2 -> GlobalAvgPool -> Dense(->tanh)。
    只依赖 numpy（无 TensorFlow），可在本机离线训练/推理；文档对应公式见 docs/04。

    用法：
        net = SimpleCNN(in_channels=3, hidden=8, out=1, img=(48,64))
        net.train(images_bhwc, labels, epochs=200)
        steer = float(net.predict(one_image_hwc)[0])

    图像输入约定：HxWxC，值域 [0,255]（内部归一化到 [0,1]）。
    """

    def __init__(self, in_channels=3, hidden=8, out=1, img=(48, 64), seed=0,
                 ksize=3, pool=2, strides=1):
        self.H, self.W = img
        self.in_c = in_channels
        self.ksize = ksize
        self.pool = pool
        self.out = out
        self.rng = np.random.RandomState(seed)

        # 两层卷积核：C0 -> hidden1 -> hidden2
        self.c1 = self._init_conv(in_channels, hidden, ksize, seed=seed)
        self.c2 = self._init_conv(hidden, hidden, ksize, seed=seed + 1)
        # 两层池化后特征图尺寸
        H1 = (self.H - ksize + 1) // pool
        W1 = (self.W - ksize + 1) // pool
        H2 = (H1 - ksize + 1) // pool
        W2 = (W1 - ksize + 1) // pool
        self.fc_flat = H2 * W2 * hidden   # 全局平均池化后为 hidden 维
        self.fc = self._init_dense(hidden, out, seed=seed + 2)

    def _init_conv(self, cin, cout, k, seed):
        rng = np.random.RandomState(seed)
        # He init
        std = np.sqrt(2.0 / (cin * k * k))
        return rng.randn(cout, cin, k, k) * std, np.zeros((cout,))

    def _init_dense(self, fin, fout, seed):
        rng = np.random.RandomState(seed)
        std = np.sqrt(2.0 / fin)
        return rng.randn(fout, fin) * std, np.zeros((fout,))

    # ---- 卷积辅助（im2col 简化：只处理 stride=1, 无 padding）----
    def _conv_forward(self, x, Wk, b):
        """x:(N,H,W,C) -> (N,H',W',C')：valid 卷积，无 padding。"""
        N, Hh, Ww, C = x.shape
        k = self.ksize
        cout = Wk.shape[0]
        Ho = Hh - k + 1
        Wo = Ww - k + 1
        out = np.zeros((N, Ho, Wo, cout))
        for i in range(Ho):
            for j in range(Wo):
                patch = x[:, i:i + k, j:j + k, :]  # (N,k,k,C)
                # Wk:(cout,cin,k,k) -> (1,cout)
                out[:, i, j, :] = np.tensordot(
                    patch, Wk, axes=([1, 2, 3], [2, 3, 1])
                ) + b
        return out

    def _conv_backward(self, x, dout, Wk):
        """返回 dW, db。x 为输入激活(cin-ch-last)，dout 为输出梯度(cout-ch-last)。"""
        k = self.ksize
        dx = np.zeros_like(x)
        dW = np.zeros_like(Wk)          # (cout, cin, k, k)
        N, Ho, Wo = dout.shape[:-1]
        for i in range(Ho):
            for j in range(Wo):
                d_ij = dout[:, i, j, :]               # (N, cout)
                patch = x[:, i:i + k, j:j + k, :]     # (N, k, k, cin)
                # dx（channel-last）：(N, k, k, cin)
                contrib = np.tensordot(d_ij, Wk, axes=([1], [0]))  # (N, cin, k, k)
                dx[:, i:i + k, j:j + k, :] += contrib.transpose(0, 2, 3, 1)
                # dW：(cout, cin, k, k) = d_ij^T (cout,N) x patch (N,k,k,cin)
                dW += np.tensordot(d_ij.T, patch, axes=([1], [0])).transpose(0, 3, 1, 2)
        db = dout.sum(axis=(0, 1, 2))   # (cout,)
        return dW, db, dx

    def _pool(self, x):
        """2x2 maxpool：x:(N,H,W,C) -> (N,H',W',C)。"""
        N, H, W, C = x.shape
        p = self.pool
        Ho, Wo = H // p, W // p
        out = x[:, :Ho * p, :Wo * p, :].reshape(N, Ho, p, Wo, p, C)
        return out.max(axis=(2, 4)), x

    def _pool_backward(self, dout, x):
        """dout 下采样梯度 + 原输入，回传到池化前。"""
        p = self.pool
        N, Ho, Wo, C = dout.shape
        dx = np.zeros_like(x)
        for i in range(Ho):
            for j in range(Wo):
                for di in range(p):
                    for dj in range(p):
                        idx = (i * p) + di
                        jdx = (j * p) + dj
                        dx[:, idx, jdx, :] = dout[:, i, j, :]
        return dx

    # ---- 前向 ----
    def forward(self, x):
        """x:(N,H,W,C) uint8 -> (out,) 线性输出。返回 (out, cache)。"""
        xf = x.astype(np.float32) / 255.0
        z1 = self._conv_forward(xf, self.c1[0], self.c1[1])
        a1 = relu(z1)
        p1, _ = self._pool(a1)                      # (N,H1,W1,C1)
        z2 = self._conv_forward(p1, self.c2[0], self.c2[1])
        a2 = relu(z2)
        p2, _ = self._pool(a2)                      # (N,H2,W2,C2)
        g = p2.mean(axis=(1, 2))                    # global avg pool (N,C2)
        fc_out = g @ self.fc[0].T + self.fc[1]      # (N,out)
        cache = (xf, a1, p1, a2, p2, g)
        return fc_out, cache

    def predict(self, x):
        if x.ndim == 3:
            x = x[None, ...]
        out, _ = self.forward(x)
        return out

    def train(self, images, targets, epochs=200, lr=0.1, batch=16, verbose=20):
        """images:(N,H,W,C) uint8, targets:(N,). 回归 MSE + relu conv + tanh fc。"""
        N = len(images)
        X = np.asarray(images, dtype=np.float32)
        Y = np.asarray(targets, dtype=np.float32).reshape(-1, 1)
        hist = []
        for ep in range(1, epochs + 1):
            idx = self.rng.permutation(N)
            losses = []
            for s in range(0, N, batch):
                b = idx[s:s + batch]
                Xb, Yb = X[b], Y[b]
                out, cache = self.forward(Xb)           # (B,out) 线性
                # 用 tanh 输出做 MSE（与 04 端到端一致）
                out_t = np.tanh(out)
                diff = out_t - Yb
                n = len(b)
                loss = float(np.mean(np.square(diff)))
                losses.append(loss)
                # 反向：loss -> tanh输出 -> fc线性输出
                d_out = 2.0 * diff / n * (1.0 - out_t ** 2)  # 经 tanh 导数
                xf, a1, p1, a2, p2, g = cache
                # fc 线性输出 d_out 对 fc 参数与 g 的梯度
                d_fcW = d_out.T @ g          # (out, features)
                d_fcb = d_out.sum(axis=0)    # (out,)
                n_units = p2.shape[1] * p2.shape[2]
                # global avg pool 反 => p2
                d_p2 = np.zeros_like(p2)
                for i in range(p2.shape[1]):
                    for j in range(p2.shape[2]):
                        d_p2[:, i, j, :] += d_out @ self.fc[0] / n_units
                # pool2 反 -> a2
                d_a2 = self._pool_backward(d_p2, a2)
                d_z2 = d_a2 * relu_grad(a2)
                # conv2 反
                dW2, db2, d_p1 = self._conv_backward(p1, d_z2, self.c2[0])
                # pool1 反 -> a1
                d_a1 = self._pool_backward(d_p1, a1)
                d_z1 = d_a1 * relu_grad(a1)
                dW1, db1, _dx = self._conv_backward(xf, d_z1, self.c1[0])
                # 更新
                self.c1 = (self.c1[0] - lr * dW1, self.c1[1] - lr * db1)
                self.c2 = (self.c2[0] - lr * dW2, self.c2[1] - lr * db2)
                self.fc = (self.fc[0] - lr * d_fcW, self.fc[1] - lr * d_fcb)
            hist.append(float(np.mean(losses)))
            if verbose and ep % verbose == 0:
                print(f"  cnn epoch {ep:3d} loss={hist[-1]:.5f}")
        return hist

    def save(self, path):
        import json, os
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w") as f:
            json.dump({
                "type": "SimpleCNN",
                "in_channels": self.in_c,
                "hidden": self.c1[0].shape[0],
                "out": self.out,
                "img": [self.H, self.W],
                "c1_W": self.c1[0].tolist(), "c1_b": self.c1[1].tolist(),
                "c2_W": self.c2[0].tolist(), "c2_b": self.c2[1].tolist(),
                "fc_W": self.fc[0].tolist(), "fc_b": self.fc[1].tolist(),
            }, f)
        return path

    @classmethod
    def load(cls, path):
        import json
        with open(path) as f:
            obj = json.load(f)
        net = cls(obj["in_channels"], obj["hidden"], obj["out"], tuple(obj["img"]))
        net.c1 = (np.asarray(obj["c1_W"], dtype=np.float32),
                  np.asarray(obj["c1_b"], dtype=np.float32))
        net.c2 = (np.asarray(obj["c2_W"], dtype=np.float32),
                  np.asarray(obj["c2_b"], dtype=np.float32))
        net.fc = (np.asarray(obj["fc_W"], dtype=np.float32),
                  np.asarray(obj["fc_b"], dtype=np.float32))
        return net


# ----------------------------------------------------------------------------
# 简易自测（离线，仅 numpy）
# ----------------------------------------------------------------------------
def _self_test():
    rng = np.random.RandomState(0)
    # 分类：两类线性可分 -> 精度应接近 1
    Xc = rng.randn(300, 4)
    Yc = (Xc[:, 0] + Xc[:, 1] > 0).astype(int)
    clf = MLPClassifier([4, 8, 2], seed=1)
    clf.train(Xc, Yc, epochs=300, lr=0.2, verbose=0)
    acc = float(np.mean(clf.predict(Xc) == Yc))
    # 回归：y = sin(x1)*0.5 + x2*0.3
    Xr = rng.randn(300, 2)
    Yr = np.sin(Xr[:, 0]) * 0.5 + Xr[:, 1] * 0.3
    pol = MLPPolicy([2, 12, 1], seed=2)
    pol.train(Xr, Yr.reshape(-1, 1), epochs=300, lr=0.1, verbose=0)
    mse = float(np.mean((pol.predict(Xr).ravel() - Yr) ** 2))
    print(f"[self-test] class acc={acc:.3f}  reg MSE={mse:.5f}")
    return acc, mse


if __name__ == "__main__":
    _self_test()
    print("nn_models OK (纯 numpy，可离线训练/推理)")
