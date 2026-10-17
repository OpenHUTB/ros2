#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""策略网络：四种主干 + 高斯策略头 + 价值头。

四种主干共用同一套输入输出契约，用于对比实验②（同一 PPO、不同网络结构）：

| arch | 主干 | 说明 |
|---|---|---|
| `mlp` | 深度图拉平 → 全连接 | 无空间先验的基线 |
| `cnn` | 3 层 stride-2 卷积 → 全局池化 | 有平移等变性 |
| `gru` | CNN → GRU → 取末状态 | 带记忆，可抑制单帧噪声 |
| `attn` | CNN → 特征图当 token → 单头自注意力 | 全局感受野 |

策略头输出高斯均值，标准差是**状态无关**的可学习参数（PPO 的常规做法：
状态相关的 σ 在早期容易把探索压死）。动作经 tanh 压到 [−1,1]，
再按环境的速度上限缩放。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .nn import (Adam, Conv2d, GRUCell, Layer, LayerNorm, Linear,
                 SelfAttention, relu)

__all__ = ["PolicyConfig", "Policy", "ARCHS"]

ARCHS = ("mlp", "cnn", "gru", "attn")
LOG_STD_MIN, LOG_STD_MAX = -3.0, 0.5


@dataclass
class PolicyConfig:
    arch: str = "cnn"
    depth_hw: Tuple[int, int] = (48, 64)
    vel_dim: int = 3
    hidden: int = 128
    action_dim: int = 3
    seed: int = 0
    log_std_init: float = -0.5

    def as_dict(self) -> dict:
        return {"arch": self.arch, "depth_hw": list(self.depth_hw),
                "vel_dim": int(self.vel_dim), "hidden": int(self.hidden),
                "action_dim": int(self.action_dim), "seed": int(self.seed),
                "log_std_init": float(self.log_std_init)}


class Policy:
    def __init__(self, cfg: Optional[PolicyConfig] = None):
        self.cfg = cfg or PolicyConfig()
        rng = np.random.default_rng(self.cfg.seed)
        h, w = self.cfg.depth_hw
        c = self.cfg
        self.enc: List[Layer] = []
        self.extra: List[Layer] = []

        if c.arch == "mlp":
            self.enc = [Linear(h * w + c.vel_dim, c.hidden, rng=rng),
                        Linear(c.hidden, c.hidden, rng=rng)]
        elif c.arch in ("cnn", "gru", "attn"):
            self.enc = [Conv2d(1, 16, 5, stride=2, rng=rng),
                        Conv2d(16, 24, 3, stride=2, rng=rng),
                        Conv2d(24, 32, 3, stride=2, rng=rng)]
            hh = (h - 5) // 2 + 1
            hh = (hh - 3) // 2 + 1
            hh = (hh - 3) // 2 + 1
            ww = (w - 5) // 2 + 1
            ww = (ww - 3) // 2 + 1
            ww = (ww - 3) // 2 + 1
            self.feat_dim = 32 * hh * ww
            self.grid_hw = (hh, ww)
            self.proj = Linear(self.feat_dim + c.vel_dim, c.hidden, rng=rng)
            self.extra = [self.proj]
            if c.arch == "gru":
                self.gru = GRUCell(c.hidden, c.hidden, rng=rng)
                self.extra.append(self.gru)
            elif c.arch == "attn":
                self.attn = SelfAttention(32, rng=rng)
                self.attn_norm = LayerNorm()
                self.hidden_layers = [Linear(c.hidden, c.hidden, rng=rng)]
                self.extra += [self.attn, self.attn_norm, *self.hidden_layers]
            else:
                self.hidden_layers = [Linear(c.hidden, c.hidden, rng=rng)]
                self.extra += self.hidden_layers
        else:
            raise KeyError("未知主干 %r，可选 %s" % (c.arch, list(ARCHS)))

        self.head_mean = Linear(c.hidden, c.action_dim, rng=rng, gain=0.01)
        self.head_value = Linear(c.hidden, 1, rng=rng)
        self.log_std = np.full(c.action_dim, float(c.log_std_init))
        self.extra += [self.head_mean, self.head_value]
        self._cache = None

    # ------------------------------------------------------------ 结构
    @property
    def layers(self) -> List[Layer]:
        return list(self.enc) + list(self.extra)

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

    def init_hidden(self, batch: int) -> np.ndarray:
        return np.zeros((batch, self.cfg.hidden))

    @property
    def std(self) -> np.ndarray:
        return np.exp(np.clip(self.log_std, LOG_STD_MIN, LOG_STD_MAX))

    # ------------------------------------------------------------ 前向
    def _trunk(self, obs: Dict[str, np.ndarray], h: np.ndarray, cache: bool):
        depth = np.asarray(obs["depth"], np.float32)
        vel = np.asarray(obs["vel"], np.float32).reshape(-1, self.cfg.vel_dim)
        # 环境单帧给 (H,W)，批处理给 (B,H,W)；不要靠 ndim 猜通道在哪一维
        if depth.ndim == 2:
            depth = depth[None]
        if depth.ndim == 3:
            depth = depth[:, None]
        x = depth / 255.0
        b = x.shape[0]
        if self.cfg.arch == "mlp":
            flat = np.concatenate([x.reshape(b, -1), vel], axis=1)
            a0 = relu(self.enc[0].forward(flat))
            a1 = relu(self.enc[1].forward(a0))
            feat = a1
            h_new = h                      # 无记忆主干：隐状态原样透传
            c = (flat, a0, a1)
        else:
            z1 = relu(self.enc[0].forward(x))
            z2 = relu(self.enc[1].forward(z1))
            z3 = relu(self.enc[2].forward(z2))
            if self.cfg.arch == "attn":
                b_, c_, hh, ww = z3.shape
                tok = z3.transpose(0, 2, 3, 1).reshape(b_, hh * ww, c_)
                tok = self.attn_norm.forward(self.attn.forward(tok) + tok)
                z3p = tok.reshape(b_, hh, ww, c_).transpose(0, 3, 1, 2)
            else:
                z3p = z3
            # 展平而不是全局平均池化：竞速需要"门在画面哪一侧"这种空间信息，
            # 池化会把位置信息抹掉，策略只能看到"画面里有没有门"。
            gap = z3p.reshape(b, -1)
            cat = np.concatenate([gap, vel], axis=1)
            f0 = relu(self.proj.forward(cat))
            if self.cfg.arch == "gru":
                h_new = self.gru.forward(f0, h)
                feat = relu(h_new)
            elif self.cfg.arch == "attn":
                h_new = h
                feat = relu(self.hidden_layers[0].forward(f0))
            else:
                h_new = h
                feat = relu(self.hidden_layers[0].forward(f0))
            c = (x, z1, z2, z3, cat, f0, h, h_new)
        mean = self.head_mean.forward(feat)
        value = self.head_value.forward(feat)[:, 0]
        if cache:
            self._cache = (c, feat)
        return mean, value, h_new

    def forward(self, obs, h=None):
        h = self.init_hidden(1) if h is None else h
        mean, value, h_new = self._trunk(obs, h, cache=True)
        return mean, value, h_new

    def act(self, obs, h=None, deterministic: bool = False, rng=None):
        mean, value, h_new = self.forward(obs, h)
        std = self.std[None, :]
        if deterministic:
            raw = mean
            logp = None
        else:
            rng = rng or np.random.default_rng()
            raw = mean + std * rng.normal(size=mean.shape)
            logp = self._logp(raw, mean)
        act = np.tanh(raw)
        if logp is not None:                       # tanh 的雅可比修正
            logp = logp - np.sum(np.log(1.0 - act ** 2 + 1e-6), axis=1)
        return act.astype(np.float32), logp, value, h_new, raw

    def _logp(self, raw: np.ndarray, mean: np.ndarray) -> np.ndarray:
        std = self.std[None, :]
        z = (raw - mean) / std
        return (-0.5 * z ** 2 - np.log(std) - 0.5 * math.log(2 * math.pi)
                ).sum(axis=1)

    # ------------------------------------------------------------ 反向
    def backward(self, d_mean: np.ndarray, d_value: np.ndarray,
                 d_log_std: np.ndarray, d_raw: Optional[np.ndarray] = None) -> None:
        """从两个头的**输出**反向传播。

        ``d_raw`` 是 tanh 雅可比带来的对高斯原始采样的梯度（PPO 的
        clip 项要用），若为空则由 ``d_mean`` 承担。
        """
        c, feat = self._cache
        dv = d_value[:, None]
        d_head_v = self.head_value.backward(dv)
        d_head_m = self.head_mean.backward(d_mean)
        d_feat = d_head_v + d_head_m
        # ReLU
        d_feat = d_feat * (feat > 0)

        if self.cfg.arch == "mlp":
            flat, a0, a1 = c
            d1 = self.enc[1].backward(d_feat) * (a0 > 0)
            self.enc[0].backward(d1)
        else:
            x, z1, z2, z3, cat, f0, h, h_new = c
            if self.cfg.arch == "gru":
                dh = d_feat
                d_f0 = self.gru.backward(dh)[0]
            elif self.cfg.arch == "attn":
                d_f0 = self.hidden_layers[0].backward(d_feat) * (f0 > 0)
            else:
                d_f0 = self.hidden_layers[0].backward(d_feat) * (f0 > 0)
            d_cat = self.proj.backward(d_f0 * (f0 > 0))
            d_gap = d_cat[:, :self.feat_dim]
            b_, c_, hh, ww = z3.shape
            dz3p = d_gap.reshape(b_, c_, hh, ww)
            if self.cfg.arch == "attn":
                tok = z3.transpose(0, 2, 3, 1).reshape(b_, hh * ww, c_)
                d_tok = dz3p.transpose(0, 2, 3, 1).reshape(b_, hh * ww, c_)
                d_att = self.attn_norm.backward(d_tok)
                d_z3 = self.attn.backward(d_att) + d_att
                d_z3 = d_z3.reshape(b_, hh, ww, c_).transpose(0, 3, 1, 2)
            else:
                d_z3 = dz3p
            d2 = self.enc[2].backward(d_z3) * (z2 > 0)
            d1 = self.enc[1].backward(d2) * (z1 > 0)
            self.enc[0].backward(d1)
        self.log_std_grad = d_log_std

    def save(self, path: str, meta: Optional[dict] = None) -> None:
        d: Dict[str, np.ndarray] = {
            "cfg": np.asarray(json.dumps(self.cfg.as_dict())),
            "meta": np.asarray(json.dumps(meta or {}, ensure_ascii=False)),
            "log_std": self.log_std}
        for i, l in enumerate(self.layers):
            for j, p in enumerate(l.params):
                d["L%d_P%d" % (i, j)] = p
        np.savez_compressed(path, **d)

    @classmethod
    def load(cls, path: str) -> "Policy":
        with np.load(path, allow_pickle=False) as d:
            cfg = json.loads(str(d["cfg"]))
            cfg["depth_hw"] = tuple(cfg["depth_hw"])
            p = cls(PolicyConfig(**cfg))
            p.log_std = np.array(d["log_std"])
            for i, l in enumerate(p.layers):
                for j in range(len(l.params)):
                    l.params[j][...] = d["L%d_P%d" % (i, j)]
        return p

    def summary(self) -> str:
        return ("Policy[%s]  参数 %d  隐层 %d  深度图 %s"
                % (self.cfg.arch, self.n_params(), self.cfg.hidden,
                   self.cfg.depth_hw))
