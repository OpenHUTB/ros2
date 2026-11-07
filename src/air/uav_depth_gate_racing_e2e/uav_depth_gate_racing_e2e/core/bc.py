#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行为克隆：把专家动作当作回归目标做监督训练。

动作先按环境的限幅归一化到 [−1,1]，策略输出层是 tanh，两者同一尺度。

损失可选 Huber 或 MSE。**默认 MSE**：Huber 在残差大于 ``delta`` 后退化成 L1，
梯度是常数，回归目标变成条件**中位数**。本任务里横向指令在 ±3 m/s 之间来回
摆动（相邻门 y 差最大 1.4 m），中位数接近 0——实测策略把横向输出压在
[−0.63, +0.90]，专家是 [−2.64, +3.00]，闭环直接飞歪。Huber 只适合那种
"少数大机动不该主导梯度"的回归，不适合双峰目标。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .nn import Adam, log_softmax
from .policy import Policy

__all__ = ["norm_scale", "normalise", "BCConfig", "train_bc", "bc_metrics"]


def norm_scale(cfg) -> np.ndarray:
    """把物理速度换算成 [−1,1] 的尺度（与环境的限幅一致）。"""
    return np.array([cfg.max_horiz, cfg.max_horiz, cfg.max_vert], float)


def normalise(act: np.ndarray, scale: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(act, float) / scale, -1.0, 1.0)


@dataclass
class BCConfig:
    epochs: int = 12
    batch: int = 32
    lr: float = 1e-3
    weight_decay: float = 1e-6
    grad_clip: float = 1.0
    loss: str = "mse"          # "mse" 或 "huber"
    huber_delta: float = 0.1
    #: 水平镜像增广。深度图左右翻转 + v_y 取反是**严格成立**的样本：相机与机体
    #: 左右对称、重力沿 z、专家律 v_y = k(目标_y − p_y) 在 y → −y 下正好变号。
    #: 数据翻倍，并且直接把"画面左 ↔ +v_y"这条符号约定教给网络。
    mirror: bool = True
    val_frac: float = 0.1
    seed: int = 0


def mirror_sample(s: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """左右镜像一份样本：深度图翻转、v_y 与 a_y 取反。"""
    return {"depth": np.ascontiguousarray(s["depth"][:, ::-1]),
            "vel": np.array([s["vel"][0], -s["vel"][1], s["vel"][2]], np.float32),
            "act": np.array([s["act"][0], -s["act"][1], s["act"][2]], np.float32)}


def _stack(samples: Sequence[Dict[str, np.ndarray]], scale: np.ndarray):
    depth = np.stack([s["depth"] for s in samples]).astype(np.float32)
    vel = np.stack([s["vel"] for s in samples]).astype(np.float32)
    act = np.stack([normalise(s["act"], scale) for s in samples]).astype(np.float32)
    return {"depth": depth, "vel": vel}, act


def _huber(d: np.ndarray, delta: float):
    """返回 (loss, d_loss/d_residual)。"""
    a = np.abs(d)
    quad = a <= delta
    loss = np.where(quad, 0.5 * d ** 2, delta * (a - 0.5 * delta))
    g = np.where(quad, d, delta * np.sign(d))
    return loss, g


def _loss(d: np.ndarray, cfg: "BCConfig"):
    """返回 (loss, d_loss/d_residual)。``d`` 是归一化尺度的残差。"""
    if cfg.loss == "mse":
        return 0.5 * d ** 2, d
    if cfg.loss == "huber":
        return _huber(d, cfg.huber_delta)
    raise KeyError("未知损失 %r，可选 mse / huber" % cfg.loss)


def train_bc(policy: Policy, samples: Sequence[Dict[str, np.ndarray]],
             scale: np.ndarray, cfg: Optional[BCConfig] = None,
             verbose: bool = True) -> Dict[str, List[float]]:
    cfg = cfg or BCConfig()
    rng = np.random.default_rng(cfg.seed)
    n = len(samples)
    # 先切验证集再增广：镜像样本与原件是同一帧的两种视角，混进验证集会泄漏
    idx = rng.permutation(n)
    n_val = max(int(n * cfg.val_frac), 1)
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    train_set = [samples[i] for i in tr_idx]
    if cfg.mirror:
        train_set = train_set + [mirror_sample(s) for s in train_set]
    opt = Adam(policy.layers, lr=cfg.lr, weight_decay=cfg.weight_decay,
               clip=cfg.grad_clip)
    hist: Dict[str, List[float]] = {"train": [], "val": [], "val_mse": []}

    order = np.arange(len(train_set))
    for ep in range(cfg.epochs):
        rng.shuffle(order)
        tot, nb = 0.0, 0
        for b in range(0, len(order), cfg.batch):
            sel = order[b:b + cfg.batch]
            if len(sel) < 2:
                continue
            obs, act = _stack([train_set[i] for i in sel], scale)
            b_ = len(sel)
            # 一次前向：直接拿 tanh(mean) 当预测，不做采样
            mean, value, _ = policy.forward(obs, policy.init_hidden(b_))
            pred = np.tanh(mean)
            d = pred - act
            loss, g = _loss(d, cfg)
            dl = g * (1.0 - pred ** 2)          # 过 tanh 的雅可比
            policy.zero_grad()
            policy.backward(dl / b_, np.zeros(b_), np.zeros(policy.cfg.action_dim))
            opt.step()
            tot += float(loss.mean())
            nb += 1
        hist["train"].append(tot / max(nb, 1))
        m = bc_metrics(policy, [samples[i] for i in val_idx], scale, cfg=cfg)
        hist["val"].append(m["loss"])
        hist["val_mse"].append(m["mse"])
        if verbose and (ep % 2 == 0 or ep == cfg.epochs - 1):
            print("  BC epoch %2d  训练 %.5f  验证 %.5f  验证 MSE %.5f"
                  % (ep, hist["train"][-1], m["loss"], m["mse"]))
    return hist


def bc_metrics(policy: Policy, samples: Sequence[Dict[str, np.ndarray]],
               scale: np.ndarray, batch: int = 64,
               cfg: Optional[BCConfig] = None) -> Dict[str, float]:
    """在给定样本上算损失与逐动作均方误差（已归一化尺度）。"""
    cfg = cfg or BCConfig()
    if not samples:
        return {"loss": float("nan"), "mse": float("nan")}
    tot, mse, nb = 0.0, 0.0, 0
    for b in range(0, len(samples), batch):
        sel = samples[b:b + batch]
        obs, act = _stack(sel, scale)
        mean, _, _ = policy.forward(obs, policy.init_hidden(len(sel)))
        pred = np.tanh(mean)
        loss, _ = _loss(pred - act, cfg)
        tot += float(loss.mean())
        mse += float(np.mean((pred - act) ** 2))
        nb += 1
    return {"loss": tot / max(nb, 1), "mse": mse / max(nb, 1),
            "n": float(len(samples))}
