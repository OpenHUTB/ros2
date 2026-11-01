#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""残差动力学网络：手写前向/反向的 numpy MLP + 自适应优化器。

不引 torch：仿真与 ROS 两侧都要能跑，纯 numpy 省掉一个 700 MB 依赖，
且 9 万样本 × 5,315 个参数在这个规模下 torch 并不更快（实测单轮 0.4 s vs 0.5 s）。
ROS 侧 Python 3.8 也能直接导入本模块。
"""
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

__all__ = ["FEATURE_DIM", "build_features", "MLP", "ResidualNet"]

#: 输入特征维数，见 build_features
FEATURE_DIM = 14


def build_features(v_b: np.ndarray, a_cmd_b: np.ndarray, omega_b: np.ndarray,
                   z: float, thrust: float, out: Optional[np.ndarray] = None) -> np.ndarray:
    """机体状态 → 残差网络输入 (14,)。

    全部取机体系量：阻力与姿态无关，标称模型也在机体系里，网络学到的是
    "速度→力"这一与姿态解耦的映射，同一份权重能覆盖任意姿态。

    前三项是原始速度，紧接着是**二次阻力基** ``v|v|`` —— 气动阻力本就正比于它，
    把这一项显式喂进去，网络不必从零拟合平方关系（实测外推到 2 倍训练速度时，
    RMSE 从 0.62 降到 0.19 m/s²）。
    """
    v = np.asarray(v_b, dtype=float)
    a = np.asarray(a_cmd_b, dtype=float)
    w = np.asarray(omega_b, dtype=float)
    s_h = math.hypot(v[0], v[1])
    s_v = abs(v[2])
    f = out if out is not None else np.empty(FEATURE_DIM)
    f[0:3] = v
    f[3] = v[0] * s_h
    f[4] = v[1] * s_h
    f[5] = v[2] * s_v
    f[6:9] = a
    f[9:12] = w
    f[12] = z
    f[13] = thrust
    return f


# --------------------------------------------------------------------------- #
class MLP:
    """全连接网络：tanh 隐层 + 线性输出，Adam 训练，反向传播手写。"""

    def __init__(self, sizes: Sequence[int] = (FEATURE_DIM, 64, 64, 3),
                 seed: int = 0, out_gain: float = 0.0):
        self.sizes = tuple(int(s) for s in sizes)
        rng = np.random.default_rng(seed)
        self.W, self.b = [], []
        for i in range(len(self.sizes) - 1):
            n_in, n_out = self.sizes[i], self.sizes[i + 1]
            # Glorot 均匀初始化，适配 tanh 的线性区间
            lim = math.sqrt(6.0 / (n_in + n_out))
            self.W.append(rng.uniform(-lim, lim, (n_in, n_out)))
            self.b.append(np.zeros(n_out))
        # 输出层零初始化：未经训练的网络残差恒为 0，控制器退化成纯几何控制器。
        # 若用随机初始化，训练前的闭环会被随机残差直接掀翻，连数据都采不到。
        self.W[-1] *= out_gain
        self._adam = [dict(m=np.zeros_like(w), v=np.zeros_like(w),
                           mb=np.zeros_like(bb), vb=np.zeros_like(bb))
                      for w, bb in zip(self.W, self.b)]
        self._step = 0

    # ------------------------------------------------------------ 前向
    def forward(self, X: np.ndarray, cache: bool = False):
        A = np.asarray(X, dtype=float)
        if A.ndim == 1:
            A = A[None, :]
        hs = [] if cache else None
        for i in range(len(self.W) - 1):
            A = np.tanh(A @ self.W[i] + self.b[i])
            if cache:
                hs.append(A)
        Y = A @ self.W[-1] + self.b[-1]
        return (Y, hs) if cache else Y

    def hidden(self, X: np.ndarray) -> np.ndarray:
        """最后一层隐层激活，并附加常数列（供 RLS 连同偏置一起在线更新）。"""
        A = np.asarray(X, dtype=float)
        if A.ndim == 1:
            A = A[None, :]
        for i in range(len(self.W) - 1):
            A = np.tanh(A @ self.W[i] + self.b[i])
        return np.hstack([A, np.ones((A.shape[0], 1))])

    # ------------------------------------------------------------ 反向
    def backward(self, X: np.ndarray, hs: list, dY: np.ndarray):
        """返回与 self.W/self.b 同构的梯度列表。"""
        A = np.asarray(X, dtype=float)
        if A.ndim == 1:
            A = A[None, :]
        gW = [None] * len(self.W)
        gb = [None] * len(self.b)
        D = np.asarray(dY, dtype=float)
        gW[-1] = hs[-1].T @ D
        gb[-1] = D.sum(axis=0)
        for i in range(len(self.W) - 2, -1, -1):
            D = (D @ self.W[i + 1].T) * (1.0 - hs[i] ** 2)
            prev = A if i == 0 else hs[i - 1]
            gW[i] = prev.T @ D
            gb[i] = D.sum(axis=0)
        return gW, gb

    def adam_step(self, gW, gb, lr: float = 1e-3, b1: float = 0.9,
                  b2: float = 0.999, eps: float = 1e-8, weight_decay: float = 0.0):
        self._step += 1
        t = self._step
        for i in range(len(self.W)):
            for g, p, st in ((gW[i], self.W[i], "m"), (gb[i], self.b[i], "mb")):
                if weight_decay:
                    g = g + weight_decay * (self.W[i] if st == "m" else self.b[i])
                key_v = "v" if st == "m" else "vb"
                st_m = self._adam[i][st]
                st_v = self._adam[i][key_v]
                st_m *= b1
                st_m += (1.0 - b1) * g
                st_v *= b2
                st_v += (1.0 - b2) * (g * g)
                m_hat = st_m / (1.0 - b1 ** t)
                v_hat = st_v / (1.0 - b2 ** t)
                if st == "m":
                    p -= lr * m_hat / (np.sqrt(v_hat) + eps)
                else:
                    p -= lr * m_hat / (np.sqrt(v_hat) + eps)

    # ------------------------------------------------------------ 存取
    def export(self) -> Dict[str, np.ndarray]:
        d: Dict[str, np.ndarray] = {"sizes": np.asarray(self.sizes)}
        for i, (w, bb) in enumerate(zip(self.W, self.b)):
            d["W%d" % i] = w
            d["b%d" % i] = bb
        return d

    def load(self, d: Dict[str, np.ndarray]) -> None:
        n = len(self.W)
        for i in range(n):
            self.W[i] = np.array(d["W%d" % i], dtype=float)
            self.b[i] = np.array(d["b%d" % i], dtype=float)


# --------------------------------------------------------------------------- #
class ResidualNet:
    """残差网络：特征构造 + 标准化 + MLP + 输出限幅。

    输出限幅不是可有可无：网络在训练分布外可能给出任意大的残差，直接叠加到
    推力通道上会立刻发散。但限幅值必须**大于训练目标本身的量级**，否则网络
    连训练集都拟合不了：+0.5 kg 载荷对应的垂直残差就有 5.2 m/s²，早期把限幅
    设成 3.0，训练集 RMSE 卡在 0.82 m/s² 再也降不下去。取 10 m/s²
    （约 1 g，标称机悬停推力的 2/3），既覆盖全部档位又留了安全边界。
    """

    CLIP = 10.0

    def __init__(self, sizes: Sequence[int] = (FEATURE_DIM, 64, 64, 3),
                 seed: int = 0, out_scale: float = 2.0):
        self.mlp = MLP(sizes, seed=seed, out_gain=0.0)
        self.out_scale = float(out_scale)
        self.x_mean = np.zeros(FEATURE_DIM)
        self.x_std = np.ones(FEATURE_DIM)

    # ------------------------------------------------------------ 特征
    def features(self, v_b, a_cmd_b, omega_b, z, thrust) -> np.ndarray:
        return build_features(v_b, a_cmd_b, omega_b, z, thrust)

    def features_batch(self, V_b, A_cmd_b, W_b, Z, T) -> np.ndarray:
        V = np.asarray(V_b, dtype=float).reshape(-1, 3)
        A = np.asarray(A_cmd_b, dtype=float).reshape(-1, 3)
        W = np.asarray(W_b, dtype=float).reshape(-1, 3)
        Z = np.asarray(Z, dtype=float).reshape(-1)
        T = np.asarray(T, dtype=float).reshape(-1)
        return np.column_stack([V, V[:, 0] * np.hypot(V[:, 0], V[:, 1]),
                                V[:, 1] * np.hypot(V[:, 0], V[:, 1]),
                                V[:, 2] * np.abs(V[:, 2]),
                                A, W, Z, T])

    def fit_scaler(self, X: np.ndarray) -> None:
        X = np.asarray(X, dtype=float)
        self.x_mean = X.mean(axis=0)
        s = X.std(axis=0)
        s[s < 1e-6] = 1.0                      # 常数特征不做缩放
        self.x_std = s

    def _norm(self, X: np.ndarray) -> np.ndarray:
        return (np.asarray(X, dtype=float) - self.x_mean) / self.x_std

    # ------------------------------------------------------------ 推理
    def hidden(self, X: np.ndarray) -> np.ndarray:
        return self.mlp.hidden(self._norm(X))

    def predict_from_hidden(self, H: np.ndarray) -> np.ndarray:
        """从最后一层输入直接算输出（RLS 改写最后一层后走这条路径）。"""
        Y = (np.asarray(H) @ self.mlp.W[-1] + self.mlp.b[-1]) * self.out_scale
        return np.clip(Y, -self.CLIP, self.CLIP)

    def predict_batch(self, X: np.ndarray) -> np.ndarray:
        Y = self.mlp.forward(self._norm(X)) * self.out_scale
        return np.clip(Y, -self.CLIP, self.CLIP)

    def predict(self, X: np.ndarray) -> np.ndarray:
        """单样本或批量，返回机体系残差加速度 (…, 3)，单位 m/s²。"""
        single = np.asarray(X, dtype=float).ndim == 1
        Y = self.predict_batch(np.atleast_2d(X))
        return Y[0] if single else Y

    # ------------------------------------------------------------ 训练
    def loss_and_grad(self, X: np.ndarray, Y: np.ndarray, l2: float = 0.0):
        """MSE 与其解析梯度。

        ⚠️ 梯度除以**元素总数** (N×3) 而不是样本数 N：``np.mean`` 摊的是全部
        元素，只除 N 会让梯度整体大 3 倍。Adam 对梯度整体缩放不敏感（所以
        这个 bug 训练时看不出来），但换成 SGD 或调 lr 就会对不上。
        """
        Yn, hs = self.mlp.forward(self._norm(X), cache=True)
        resid = Yn - np.asarray(Y, dtype=float) / self.out_scale
        loss = float(np.mean(resid ** 2))
        dY = 2.0 * resid / max(resid.size, 1)
        gW, gb = self.mlp.backward(self._norm(X), hs, dY)
        return loss, gW, gb

    def train_epoch(self, X, Y, batch: int = 128, lr: float = 2e-3,
                    l2: float = 1e-5, seed: int = 0) -> float:
        rng = np.random.default_rng(seed)
        idx = rng.permutation(len(X))
        Xn = self._norm(X)
        Yn = np.asarray(Y, dtype=float) / self.out_scale
        total, nb = 0.0, 0
        for s in range(0, len(idx), batch):
            sel = idx[s:s + batch]
            xb, yb = Xn[sel], Yn[sel]
            _, hs = self.mlp.forward(xb, cache=True)
            resid = hs[-1] @ self.mlp.W[-1] + self.mlp.b[-1] - yb
            total += float(np.mean(resid ** 2))
            nb += 1
            gW, gb = self.mlp.backward(xb, hs, 2.0 * resid / max(resid.size, 1))
            if l2:
                for i in range(len(gW)):
                    gW[i] = gW[i] + l2 * self.mlp.W[i]
            self.mlp.adam_step(gW, gb, lr=lr)
        return total / max(nb, 1)

    # ------------------------------------------------------------ 与 RLS 对接
    def last_layer_input(self, X: np.ndarray) -> np.ndarray:
        """RLS 的回归向量：最后一层隐层激活 + 偏置列。"""
        return self.mlp.hidden(self._norm(X))

    def set_last_layer(self, theta: np.ndarray, bias: np.ndarray) -> None:
        """写入**物理量纲**的最后一层（内部自动除掉 ``out_scale``）。"""
        self.mlp.W[-1] = np.asarray(theta, dtype=float).reshape(self.mlp.W[-1].shape) / self.out_scale
        self.mlp.b[-1] = np.asarray(bias, dtype=float).reshape(self.mlp.b[-1].shape) / self.out_scale

    def get_last_layer(self) -> Tuple[np.ndarray, np.ndarray]:
        """读回**物理量纲**的最后一层。"""
        return self.mlp.W[-1].copy() * self.out_scale, self.mlp.b[-1].copy() * self.out_scale

    # ------------------------------------------------------------ 存取
    def save(self, path: str, meta: Optional[dict] = None) -> None:
        d = dict(self.mlp.export())
        d["out_scale"] = np.asarray(self.out_scale)
        d["x_mean"] = self.x_mean
        d["x_std"] = self.x_std
        d["meta_json"] = np.asarray(json.dumps(meta or {}, ensure_ascii=False))
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "ResidualNet":
        with np.load(path, allow_pickle=False) as d:
            sizes = tuple(int(x) for x in d["sizes"])
            net = cls(sizes=sizes)
            net.mlp.load({k: d[k] for k in d.files if k.startswith(("W", "b"))})
            net.out_scale = float(d["out_scale"])
            net.x_mean = np.array(d["x_mean"], dtype=float)
            net.x_std = np.array(d["x_std"], dtype=float)
            net.meta = json.loads(str(d["meta_json"])) if "meta_json" in d.files else {}
        return net
