#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RGB → 语义分割的小型编解码网络，纯 numpy。

结构：三层 stride-2 卷积下采样到 1/8，1×1 卷积出类别 logits，再最近邻上采样
回原分辨率。上采样放在 logits 上而不是特征图上，解码器不需要参数，输入尺寸
也不必是 2 的幂；损失按全分辨率算，梯度回传到 1/8 分辨率只有一次块求和。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .nn import Adam, Conv2d, Layer, softmax

__all__ = ["SegConfig", "ResizeNearest", "SegNet", "class_weights"]


@dataclass
class SegConfig:
    #: 输入尺寸（高、宽）。纯 numpy 的 im2col/col2im 在 96×96 上单轮要 6 分钟，
    #: 64×64 是 CPU 训练速度与分割精度的折中
    in_hw: Tuple[int, int] = (64, 64)
    n_class: int = 9
    ch: Tuple[int, int, int] = (16, 24, 32)
    seed: int = 0

    def as_dict(self) -> dict:
        return {"in_hw": list(self.in_hw), "n_class": int(self.n_class),
                "ch": list(self.ch), "seed": int(self.seed)}


class ResizeNearest(Layer):
    """最近邻缩放，目标尺寸任意。反向按源像素聚合同一目标的全部梯度。"""

    params: List[np.ndarray] = []
    grads: List[np.ndarray] = []

    def __init__(self, out_hw: Tuple[int, int]):
        self.out_hw = (int(out_hw[0]), int(out_hw[1]))
        self._cache = None

    def _index(self, src: int, dst: int) -> np.ndarray:
        return (np.arange(dst) * src) // dst

    def forward(self, x: np.ndarray) -> np.ndarray:
        n, c, h, w = x.shape
        oh, ow = self.out_hw
        ih, iw = self._index(h, oh), self._index(w, ow)
        self._cache = (n, c, h, w, ih, iw)
        return x[:, :, ih][:, :, :, iw]

    def backward(self, dy: np.ndarray) -> np.ndarray:
        n, c, h, w, ih, iw = self._cache
        dx = np.zeros((n, c, h, w))
        flat_src = (ih[:, None] * w + iw[None, :]).ravel()
        d = dy.reshape(n, c, -1)
        out = np.zeros((n, c, h * w))
        np.add.at(out, (slice(None), slice(None), flat_src), d)
        return out.reshape(n, c, h, w)


class SegNet:
    """编码器 + 1×1 分类头 + 上采样。"""

    def __init__(self, cfg: Optional[SegConfig] = None):
        self.cfg = cfg or SegConfig()
        rng = np.random.default_rng(self.cfg.seed)
        c1, c2, c3 = self.cfg.ch
        h, w = self.cfg.in_hw
        self.c1 = Conv2d(3, c1, 5, stride=2, rng=rng)
        self.c2 = Conv2d(c1, c2, 3, stride=2, rng=rng)
        self.c3 = Conv2d(c2, c3, 3, stride=2, rng=rng)
        self.head = Conv2d(c3, self.cfg.n_class, 1, stride=1, rng=rng)
        # 编码后尺寸（valid 卷积，不补零）
        hh = (h - 5) // 2 + 1
        hh = (hh - 3) // 2 + 1
        hh = (hh - 3) // 2 + 1
        ww = (w - 5) // 2 + 1
        ww = (ww - 3) // 2 + 1
        ww = (ww - 3) // 2 + 1
        self.up = ResizeNearest((h, w))
        self.layers: List[Layer] = [self.c1, self.c2, self.c3, self.head, self.up]
        self._a1 = self._a2 = self._a3 = None

    # ------------------------------------------------------------ 结构
    @property
    def params(self):
        return [p for l in self.layers for p in l.params]

    @property
    def grads(self):
        return [g for l in self.layers for g in l.grads]

    def n_params(self) -> int:
        return int(sum(p.size for p in self.params))

    def zero_grad(self) -> None:
        for g in self.grads:
            g[...] = 0.0

    def _relu(self, x):
        m = x > 0
        return x * m, m

    # ------------------------------------------------------------ 前向
    def forward(self, rgb: np.ndarray) -> np.ndarray:
        """``rgb`` 为 (N,3,H,W) 或 (N,H,W,3) float，取值 [0,1]。返回 (N,C,H,W) logits。

        上采样的目标尺寸取自**本次输入**的空间尺寸，而不是构造时的 ``cfg``：
        同一份权重经常要在不同分辨率上跑（训练 96×96、板端 64×64），把目标
        尺寸钉死在构造时会让 logits 与标签尺寸对不上。
        """
        x = np.transpose(rgb, (0, 3, 1, 2)) if rgb.shape[-1] == 3 else rgb
        self.up.out_hw = (x.shape[2], x.shape[3])
        z1, self._a1 = self._relu(self.c1.forward(x))
        z2, self._a2 = self._relu(self.c2.forward(z1))
        z3, self._a3 = self._relu(self.c3.forward(z2))
        coarse = self.head.forward(z3)
        return self.up.forward(coarse)

    def backward(self, d_logits: np.ndarray) -> None:
        d = self.up.backward(d_logits)
        d = self.head.backward(d)
        d = self.c3.backward(d * self._a3)
        d = self.c2.backward(d * self._a2)
        self.c1.backward(d * self._a1)

    # ------------------------------------------------------------ 推理
    def predict(self, rgb: np.ndarray) -> np.ndarray:
        """返回 (N,H,W) int32 类别索引。"""
        return np.argmax(self.forward(rgb), axis=1).astype(np.int32)

    def predict_proba(self, rgb: np.ndarray) -> np.ndarray:
        """返回 (N,C,H,W) 类别概率。"""
        lg = self.forward(rgb)
        return softmax(lg, axis=1)

    # ------------------------------------------------------------ 损失
    def loss_and_grad(self, rgb: np.ndarray, target: np.ndarray,
                      weight: Optional[np.ndarray] = None,
                      ignore_index: int = -1) -> Tuple[float, np.ndarray]:
        """逐像素加权交叉熵。``target`` 为 (N,H,W) 类别索引。"""
        lg = self.forward(rgb)
        n, c, h, w = lg.shape
        p = softmax(lg, axis=1)
        t = np.asarray(target).astype(np.int64)
        keep = (t != ignore_index)
        t = np.where(keep, t, 0)
        n_keep = int(keep.sum())
        if n_keep == 0:
            return 0.0, np.zeros_like(lg)

        onehot = np.zeros((n, c, h, w))
        onehot[np.arange(n)[:, None, None], t,
               np.arange(h)[None, :, None], np.arange(w)[None, None, :]] = 1.0
        mask = keep[:, None, :, :].astype(float)
        if weight is not None:
            wgt = np.asarray(weight, float).reshape(1, c, 1, 1)
            onehot = onehot * wgt
        loss = float(-np.sum(np.log(np.maximum(
            (p * onehot).sum(axis=1, keepdims=True), 1e-12)) * mask) / n_keep)

        d = (p * onehot.sum(axis=1, keepdims=True) - onehot) * mask / n_keep
        self.backward(d)
        return loss, d

    # ------------------------------------------------------------ 存取
    def save(self, path: str, meta: Optional[dict] = None) -> None:
        d: Dict[str, np.ndarray] = {
            "cfg": np.asarray(json.dumps(self.cfg.as_dict())),
            "meta": np.asarray(json.dumps(meta or {}, ensure_ascii=False))}
        for i, l in enumerate(self.layers):
            for j, p in enumerate(l.params):
                d["L%d_P%d" % (i, j)] = p
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "SegNet":
        with np.load(path, allow_pickle=False) as d:
            cfg = json.loads(str(d["cfg"]))
            cfg["in_hw"] = tuple(cfg["in_hw"])
            cfg["ch"] = tuple(cfg["ch"])
            net = cls(SegConfig(**cfg))
            for i, l in enumerate(net.layers):
                for j in range(len(l.params)):
                    l.params[j][...] = d["L%d_P%d" % (i, j)]
        return net

    def summary(self) -> str:
        h, w = self.cfg.in_hw
        lines = ["SegNet 输入 %dx%d  类别 %d  参数 %d"
                 % (w, h, self.cfg.n_class, self.n_params())]
        for name, l in (("enc1", self.c1), ("enc2", self.c2),
                        ("enc3", self.c3), ("head", self.head)):
            lines.append("  %-5s W%s" % (name, l.W.shape))
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
def class_weights(counts: np.ndarray, power: float = 0.5) -> np.ndarray:
    """按类别频率的逆平方根加权，缓解"地面占多数像素"的类别不平衡。

    ``counts`` 为各类别像素数。``power=0`` 退化为不加权。
    """
    c = np.asarray(counts, float)
    w = np.where(c > 0, (c.sum() / np.maximum(c, 1.0)) ** power, 0.0)
    keep = w > 0
    if not np.any(keep):
        return np.ones_like(w)
    w[keep] /= w[keep].mean()
    return w
