#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线训练：标准化 → 小批量 Adam → 按验证集早停。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .dataset import Dataset
from .residual import ResidualNet

__all__ = ["TrainConfig", "train_residual", "evaluate_residual"]


@dataclass
class TrainConfig:
    hidden: Tuple[int, ...] = (64, 64)
    epochs: int = 60
    batch: int = 128
    lr: float = 2e-3
    l2: float = 1e-5
    val_frac: float = 0.2
    patience: int = 12
    seed: int = 0


def train_residual(ds: Dataset, cfg: Optional[TrainConfig] = None,
                   verbose: bool = True) -> Tuple[ResidualNet, Dict[str, List[float]]]:
    cfg = cfg or TrainConfig()
    tr, va = ds.split(cfg.val_frac, seed=cfg.seed)

    net = ResidualNet(sizes=(ds.X.shape[1],) + tuple(cfg.hidden) + (3,), seed=cfg.seed)
    net.fit_scaler(tr.X)

    # 两条曲线都换算成**物理量纲**的 MSE（(m/s²)²）再记录：
    # train_epoch 返回的是归一化空间的 MSE，predict_batch 返回的已是 m/s²，
    # 两者混在一起画会让验证曲线凭空高 4 倍。
    hist: Dict[str, List[float]] = {"train": [], "val": []}
    best, best_epoch, best_W = float("inf"), -1, None
    for ep in range(cfg.epochs):
        loss = net.train_epoch(tr.X, tr.Y, batch=cfg.batch, lr=cfg.lr,
                               l2=cfg.l2, seed=cfg.seed + ep)
        v = float(np.mean((net.predict_batch(va.X) - va.Y) ** 2))
        hist["train"].append(loss * net.out_scale ** 2)
        hist["val"].append(v)
        if v < best - 1e-9:
            best, best_epoch = v, ep
            best_W = [w.copy() for w in net.mlp.W]
        elif ep - best_epoch >= cfg.patience:
            if verbose:
                print("  验证损失 %d 轮未改善，在第 %d 轮早停" % (cfg.patience, ep))
            break
        if verbose and (ep % 10 == 0 or ep == cfg.epochs - 1):
            print("  epoch %3d  训练 RMSE=%.4f  验证 RMSE=%.4f  (最优 %.4f @%d)"
                  % (ep, np.sqrt(hist["train"][-1]), np.sqrt(v),
                     np.sqrt(best), best_epoch))
    if best_W is not None:
        net.mlp.W = best_W
    hist["best_val"] = best
    hist["best_epoch"] = best_epoch
    return net, hist


def evaluate_residual(net: ResidualNet, ds: Dataset, settle: int = 0) -> Dict[str, float]:
    """在数据集上评估残差预测精度（返回与 ``residual_metrics`` 同名的键）。"""
    from .metrics import residual_metrics
    hat = net.predict_batch(ds.X)
    return residual_metrics(ds.Y, hat, settle=settle)
