"""逐点分类用的多层感知机（纯 NumPy 实现）。

为什么不用 PyTorch：模型的输入只有 9 维特征，网络两层 32 个隐单元，参数量一万出头，
用 NumPy 手写前向/反向足够快，训练完的权重是几百 KB 的 ``.npz``，
符合"仓库只存文本和少量数据"的约定，也方便离线跑单测。

结构：``输入 -> 全连接 -> ReLU -> 全连接 -> ReLU -> 全连接 -> softmax``。
损失用加权交叉熵（类别不平衡时按 :func:`labels.class_weights` 加权），
优化器是带动量的小批量梯度下降。
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Sequence, Tuple

import numpy as np


def train_val_split(features: np.ndarray, labels: np.ndarray, val_ratio: float = 0.2,
                    seed: int = 0) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """按比例随机划分训练/验证集，返回 ``(X_tr, y_tr, X_val, y_val)``。"""
    feats = np.asarray(features, dtype=float)
    labs = np.asarray(labels, dtype=int)
    if feats.shape[0] != labs.shape[0]:
        raise ValueError('特征与标签数量不一致')
    if not 0.0 <= val_ratio < 1.0:
        raise ValueError('验证集比例应在 [0, 1) 内')
    n = feats.shape[0]
    n_val = int(round(n * val_ratio))
    if n_val <= 0 or n_val >= n:
        raise ValueError('样本太少（%d 个），无法划分验证集' % n)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(n)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    return feats[tr_idx], labs[tr_idx], feats[val_idx], labs[val_idx]


class MLP:
    """三层全连接网络，softmax 输出，加权交叉熵损失。"""

    def __init__(self, n_in: int, hidden: Sequence[int] = (32, 32),
                 n_out: int = 4, seed: int = 0):
        if n_in <= 0 or n_out <= 0:
            raise ValueError('输入/输出维度必须为正')
        self.n_in = int(n_in)
        self.hidden = tuple(int(h) for h in hidden)
        self.n_out = int(n_out)
        rng = np.random.default_rng(seed)
        sizes = (self.n_in,) + self.hidden + (self.n_out,)
        self.params: Dict[str, np.ndarray] = {}
        for i in range(len(sizes) - 1):
            # He 初始化：标准差 sqrt(2 / fan_in)，适合 ReLU
            scale = np.sqrt(2.0 / sizes[i])
            self.params['W%d' % i] = rng.normal(0.0, scale, size=(sizes[i], sizes[i + 1]))
            self.params['b%d' % i] = np.zeros(sizes[i + 1])

    # ------------------------------------------------------------------ 前向
    def forward(self, features: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """返回 ``(logits, 每层激活值)``，激活值用于反向传播。"""
        x = np.asarray(features, dtype=float)
        if x.ndim != 2 or x.shape[1] != self.n_in:
            raise ValueError('输入应为 (N, %d)，实际 %s' % (self.n_in, (x.shape,)))
        acts = [x]
        n_layers = len(self.hidden) + 1
        for i in range(n_layers - 1):
            x = np.maximum(0.0, x @ self.params['W%d' % i] + self.params['b%d' % i])
            acts.append(x)
        logits = x @ self.params['W%d' % (n_layers - 1)] + self.params['b%d' % (n_layers - 1)]
        return logits, acts

    def predict_proba(self, features: np.ndarray) -> np.ndarray:
        """softmax 概率。"""
        logits, _ = self.forward(features)
        return _softmax(logits)

    def predict(self, features: np.ndarray) -> np.ndarray:
        """取概率最大的类别。"""
        return np.argmax(self.predict_proba(features), axis=1)

    # ------------------------------------------------------------------ 训练
    def fit(self, features: np.ndarray, labels: np.ndarray, epochs: int = 30,
            batch_size: int = 256, lr: float = 0.01, momentum: float = 0.9,
            weight_decay: float = 1e-4, sample_weights: Sequence[float] | None = None,
            seed: int = 0) -> Dict[str, List[float]]:
        """小批量训练，返回每轮的 ``{'loss': [...], 'acc': [...]}``。"""
        x = np.asarray(features, dtype=float)
        y = np.asarray(labels, dtype=int)
        if x.shape[0] != y.shape[0] or x.shape[0] == 0:
            raise ValueError('训练数据为空或数量不一致')
        if np.any(y < 0) or np.any(y >= self.n_out):
            raise ValueError('标签超出 [0, %d) 范围' % self.n_out)

        class_w = (np.asarray(sample_weights, dtype=float)
                   if sample_weights is not None else np.ones(self.n_out))
        if class_w.size != self.n_out:
            raise ValueError('类别权重数量应为 %d' % self.n_out)
        class_w = np.where(class_w > 0, class_w, 0.0)

        rng = np.random.default_rng(seed)
        velocity = {k: np.zeros_like(v) for k, v in self.params.items()}
        history: Dict[str, List[float]] = {'loss': [], 'acc': []}
        n = x.shape[0]

        for _ in range(epochs):
            order = rng.permutation(n)
            epoch_loss, correct = 0.0, 0
            for start in range(0, n, batch_size):
                idx = order[start:start + batch_size]
                xb, yb = x[idx], y[idx]
                logits, acts = self.forward(xb)
                probs = _softmax(logits)
                w = class_w[yb]

                # 数值稳定的加权交叉熵
                picked = probs[np.arange(xb.shape[0]), yb]
                loss = -np.sum(w * np.log(np.clip(picked, 1e-12, 1.0))) / max(w.sum(), 1e-12)
                epoch_loss += loss * xb.shape[0]

                # softmax + 交叉熵的梯度
                grad = probs.copy()
                grad[np.arange(xb.shape[0]), yb] -= 1.0
                grad *= (w / max(w.sum(), 1e-12))[:, None]

                n_layers = len(self.hidden) + 1
                for i in range(n_layers - 1, -1, -1):
                    act_in = acts[i]
                    gW = act_in.T @ grad + weight_decay * self.params['W%d' % i]
                    gb = grad.sum(axis=0)
                    velocity['W%d' % i] = momentum * velocity['W%d' % i] - lr * gW
                    velocity['b%d' % i] = momentum * velocity['b%d' % i] - lr * gb
                    self.params['W%d' % i] += velocity['W%d' % i]
                    self.params['b%d' % i] += velocity['b%d' % i]
                    if i > 0:
                        grad = (grad @ self.params['W%d' % i].T) * (act_in > 0)

                correct += int(np.sum(np.argmax(probs, axis=1) == yb))

            history['loss'].append(round(epoch_loss / n, 6))
            history['acc'].append(round(correct / n, 6))
        return history

    # ------------------------------------------------------------------ 存取
    def save(self, path: str, meta: Dict[str, object] | None = None) -> None:
        """权重存成 ``.npz``；元信息（特征名、类别名等）存进同目录的 ``.json``。"""
        directory = os.path.dirname(os.path.abspath(path))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory, exist_ok=True)
        arrays = dict(self.params)
        arrays['shape'] = np.array([self.n_in] + list(self.hidden) + [self.n_out])
        np.savez_compressed(path, **arrays)
        if meta is not None:
            base = os.path.splitext(path)[0]
            with open(base + '.json', 'w', encoding='utf-8') as handle:
                json.dump(meta, handle, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str, meta: Dict[str, object] | None = None) -> 'MLP':
        """从 ``.npz`` 读回权重；不传 ``meta`` 时按权重形状推断结构。"""
        with np.load(path) as data:
            shape = [int(v) for v in data['shape']]
            model = cls(shape[0], hidden=shape[1:-1], n_out=shape[-1])
            for key in list(model.params):
                model.params[key] = np.array(data[key])
        return model

    def n_parameters(self) -> int:
        """参数总量，用于在文档里报告模型规模。"""
        return int(sum(v.size for v in self.params.values()))


def _softmax(logits: np.ndarray) -> np.ndarray:
    """数值稳定的 softmax（先减每行最大值）。"""
    z = np.asarray(logits, dtype=float)
    z = z - z.max(axis=1, keepdims=True)
    exp = np.exp(z)
    return exp / np.clip(exp.sum(axis=1, keepdims=True), 1e-12, None)