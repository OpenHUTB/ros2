#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行为克隆训练：按 episode 切验证集，MSE + Adam，保留验证最优权重。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .network import GuardNet, NetConfig

__all__ = ["TrainConfig", "build_xy", "split_episodes", "fit_norm",
           "train_policy", "offline_metrics"]


@dataclass
class TrainConfig:
    epochs: int = 120
    batch: int = 256
    lr: float = 2e-3
    weight_decay: float = 1e-4
    val_frac: float = 0.12
    seed: int = 0
    mirror: bool = True
    temporal: bool = True
    verbose: bool = True
    log_every: int = 20


def build_xy(d: Dict[str, np.ndarray], temporal: bool = True
             ) -> Tuple[np.ndarray, np.ndarray]:
    """原始记录 → (X, Y)；X 末尾三列是标称指令，Y 是教师修正指令。"""
    from .samples import build_xy as _build
    return _build(d, temporal=temporal)


def split_episodes(episode: np.ndarray, val_frac: float,
                   rng: np.random.Generator) -> Tuple[np.ndarray, np.ndarray]:
    """按 episode 切分：同一集的相邻帧高度相关，不能随机切帧。"""
    eps = np.unique(np.asarray(episode))
    if len(eps) <= 1:
        idx = np.arange(len(episode))
        return idx, idx
    n_val = max(1, int(round(len(eps) * float(val_frac))))
    perm = rng.permutation(len(eps))
    val_eps = set(eps[perm[:n_val]].tolist())
    mask = np.asarray([int(e) in val_eps for e in episode])
    return np.where(~mask)[0], np.where(mask)[0]


def fit_norm(net: GuardNet, X: np.ndarray) -> None:
    mu = np.mean(X, axis=0)
    sd = np.std(X, axis=0)
    sd = np.where(sd < 1e-4, 1.0, sd)
    net.set_norm(mu, sd)


def train_policy(net: GuardNet, X: np.ndarray, Y: np.ndarray,
                 episode: np.ndarray, cfg: Optional[TrainConfig] = None
                 ) -> Dict[str, List[float]]:
    """训练并返回逐轮损失；验证集损失最优的权重会被保留。"""
    cfg = cfg or TrainConfig()
    rng = np.random.default_rng(cfg.seed)
    tr, va = split_episodes(episode, cfg.val_frac, rng)
    fit_norm(net, X[tr])
    opt = net.optimizer(lr=cfg.lr, weight_decay=cfg.weight_decay)
    hist: Dict[str, List[float]] = {"train": [], "val": [], "val_epoch": []}
    best = float("inf")
    best_W = [l.W.copy() for l in net.layers]
    best_b = [l.b.copy() for l in net.layers]
    n = len(tr)
    steps = max(n // cfg.batch, 1)
    for ep in range(int(cfg.epochs)):
        order = rng.permutation(n)
        tot = 0.0
        for s in range(steps):
            b = tr[order[s * cfg.batch:(s + 1) * cfg.batch]]
            xb, yb = X[b], Y[b]
            pred, masks = net.forward(xb, cache=True)
            diff = pred - yb
            tot += float(np.mean(np.sum(diff ** 2, axis=1)))
            net.backward(2.0 * diff / diff.size, masks)
            opt.step()
        hist["train"].append(tot / max(steps, 1))
        if cfg.epochs > 0 and ((ep + 1) % cfg.log_every == 0
                               or ep == cfg.epochs - 1):
            v = offline_metrics(net, X[va], Y[va])["mse"]
            hist["val"].append(float(v))
            hist["val_epoch"].append(float(ep + 1))
            if v < best:
                best = float(v)
                best_W = [l.W.copy() for l in net.layers]
                best_b = [l.b.copy() for l in net.layers]
            if cfg.verbose:
                print("  [train] epoch %3d/%d train=%.5f val=%.5f"
                      % (ep + 1, cfg.epochs, hist["train"][-1], v))
    if np.isfinite(best):
        for l, W, b in zip(net.layers, best_W, best_b):
            l.W = W
            l.b = b
    hist["best_val"] = [float(best)]
    return hist


def offline_metrics(net: GuardNet, X: np.ndarray, Y: np.ndarray,
                    batch: int = 2048) -> Dict[str, float]:
    """离线指标：MSE / MAE / 方向余弦 / 与标称指令的偏差。"""
    if len(X) == 0:
        return {"mse": float("nan"), "mae": float("nan"),
                "cos": float("nan"), "bias": float("nan")}
    preds = []
    for i in range(0, len(X), int(batch)):
        preds.append(net.forward(X[i:i + batch]))
    P = np.concatenate(preds, axis=0)
    diff = P - Y
    cos = np.sum(P * Y, axis=1) / np.maximum(
        np.linalg.norm(P, axis=1) * np.linalg.norm(Y, axis=1), 1e-9)
    return {"mse": float(np.mean(np.sum(diff ** 2, axis=1))),
            "mae": float(np.mean(np.abs(diff))),
            "cos": float(np.mean(cos)),
            "bias": float(np.mean(np.abs(P)))}
