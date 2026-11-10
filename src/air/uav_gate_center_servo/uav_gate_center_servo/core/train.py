#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练小 MLP：镜像增强、Adam、早停。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .dataset import Dataset, mirror_batch
from .mlp import TinyMLP

__all__ = ["TrainConfig", "train_mlp", "evaluate_regression", "regression_metrics"]


@dataclass
class TrainConfig:
    epochs: int = 200
    batch: int = 64
    lr: float = 1e-3
    weight_decay: float = 1e-4
    patience: int = 20
    min_delta: float = 1e-4
    mirror_p: float = 0.5
    seed: int = 0


def regression_metrics(pred: np.ndarray, label: np.ndarray) -> Dict[str, float]:
    p = np.asarray(pred, np.float32).reshape(-1, 3)
    y = np.asarray(label, np.float32).reshape(-1, 3)
    err = p - y
    rmse = np.sqrt(np.mean(err * err, axis=0))
    return {
        "rmse_forward": float(rmse[0]),
        "rmse_left": float(rmse[1]),
        "rmse_up": float(rmse[2]),
        "rmse_distance": float(np.sqrt(np.mean(np.sum(err * err, axis=1)))),
    }


def evaluate_regression(model: TinyMLP, depth: np.ndarray,
                        label: np.ndarray) -> Dict[str, float]:
    return regression_metrics(model.predict(depth), label)


def _best_copy(model: TinyMLP):
    return (model.W1.copy(), model.b1.copy(), model.W2.copy(), model.b2.copy())


def _restore(model: TinyMLP, best) -> None:
    model.W1, model.b1, model.W2, model.b2 = [np.array(x, copy=True) for x in best]


def train_mlp(ds: Dataset, cfg: TrainConfig = None,
              verbose: bool = True) -> Tuple[TinyMLP, Dict[str, List[float]]]:
    cfg = cfg or TrainConfig()
    model = TinyMLP(n_in=int(np.prod(ds.train_depth.shape[1:])),
                    n_hid=32, n_out=3, seed=cfg.seed)
    model.set_stats(ds.train_depth, ds.train_label)
    rng = np.random.default_rng(cfg.seed)
    hist: Dict[str, List[float]] = {
        "epoch": [], "train_mse": [], "train_rmse_distance": [],
        "val_rmse_forward": [], "val_rmse_left": [], "val_rmse_up": [],
        "val_rmse_distance": [],
    }
    best_val = float("inf")
    best_epoch = -1
    best = _best_copy(model)
    for ep in range(int(cfg.epochs)):
        order = rng.permutation(len(ds.train_depth))
        total, nb = 0.0, 0
        for s in range(0, len(order), cfg.batch):
            sel = order[s:s + cfg.batch]
            d = ds.train_depth[sel]
            l = ds.train_label[sel]
            mask = rng.random(len(sel)) < cfg.mirror_p
            d, l = mirror_batch(d, l, mask)
            total += model.train_batch(d, l, lr=cfg.lr,
                                       weight_decay=cfg.weight_decay)
            nb += 1
        train_rmse = evaluate_regression(model, ds.train_depth,
                                         ds.train_label)["rmse_distance"]
        val = evaluate_regression(model, ds.val_depth, ds.val_label)
        hist["epoch"].append(float(ep))
        hist["train_mse"].append(total / max(nb, 1))
        hist["train_rmse_distance"].append(train_rmse)
        for k in ("rmse_forward", "rmse_left", "rmse_up", "rmse_distance"):
            hist["val_" + k].append(val[k])
        if val["rmse_distance"] < best_val - cfg.min_delta:
            best_val = val["rmse_distance"]
            best_epoch = ep
            best = _best_copy(model)
        elif ep - best_epoch >= cfg.patience:
            if verbose:
                print("  验证距离 RMSE %d 轮未改善，早停于第 %d 轮" % (cfg.patience, ep))
            break
        if verbose and (ep % 10 == 0 or ep == cfg.epochs - 1):
            print("  epoch %3d  train RMSE=%.3f  val RMSE=%.3f  best=%.3f@%d"
                  % (ep, train_rmse, val["rmse_distance"], best_val, best_epoch))
    _restore(model, best)
    hist["best_epoch"] = best_epoch
    hist["best_val_distance"] = best_val
    return model, hist
