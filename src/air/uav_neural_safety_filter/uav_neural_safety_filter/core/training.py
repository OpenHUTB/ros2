#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全网络的训练与离线评估。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from .dataset import (dataset_features, dataset_q_inputs, dataset_q_labels,
                      dataset_ranges)
from .ranges import lidar_lower_bound
from .network import SafetyNet

__all__ = ["TrainConfig", "train", "evaluate_predictions", "split_by_episode",
           "PolicyTrainConfig", "train_policy"]


@dataclass
class TrainConfig:
    epochs: int = 60
    batch: int = 256
    lr: float = 2e-3
    weight_decay: float = 1e-6
    val_frac: float = 0.12
    near_weight: float = 2.0     # 近障碍样本的过采样强度
    seed: int = 0
    verbose: bool = True


def split_by_episode(d: Dict[str, np.ndarray], val_frac: float,
                     seed: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    eps = np.unique(d["ep"])
    rng = np.random.default_rng(seed)
    rng.shuffle(eps)
    n_val = max(1, int(round(len(eps) * val_frac)))
    val = np.isin(d["ep"], eps[:n_val])
    return np.where(~val)[0], np.where(val)[0]


def _copy_params(net: SafetyNet):
    return [(p.copy(), g.copy()) for l in net.layers for p, g in zip(l.params, l.grads)]


def _restore_params(net: SafetyNet, snap) -> None:
    k = 0
    for l in net.layers:
        for p in l.params:
            p[...] = snap[k][0]
            k += 1


def _batch_loss(net, X, A, yh, yq):
    h, q, cache = net.forward(X, A)
    dh = h - yh
    dq = q - yq
    return float(np.mean(dh ** 2)), float(np.mean(dq ** 2)), dh, dq, cache


def train(net: SafetyNet, d: Dict[str, np.ndarray],
          cfg: TrainConfig) -> Dict[str, list]:
    X = dataset_features(d)
    A = dataset_q_inputs(d)
    yh = d["h"].astype(np.float64)
    yq = dataset_q_labels(d)
    tr, va = split_by_episode(d, cfg.val_frac, cfg.seed)
    net.set_norm(X[tr].mean(axis=0), X[tr].std(axis=0))
    opt = net.optimizer(cfg.lr, cfg.weight_decay)
    rng = np.random.default_rng(cfg.seed)
    hist = {"train_h": [], "train_q": [], "val_h": [], "val_q": [], "epoch": []}
    best = (float("inf"), None)
    for ep in range(cfg.epochs):
        w = 1.0 + cfg.near_weight * np.exp(-np.maximum(yh[tr], 0.0) / 0.5)
        idx = rng.choice(tr, size=len(tr), replace=True, p=w / w.sum())
        th = tq = n = 0.0
        for s in range(0, len(idx), cfg.batch):
            sel = idx[s:s + cfg.batch]
            lh, lq, dh, dq, cache = _batch_loss(net, X[sel], A[sel],
                                                yh[sel], yq[sel])
            opt.zero_grad()
            net.backward(2.0 * dh / len(sel), 2.0 * dq / len(sel), cache)
            opt.step()
            th += lh * len(sel)
            tq += lq * len(sel)
            n += len(sel)
        vh, vq, _, _, _ = _batch_loss(net, X[va], A[va], yh[va], yq[va])
        hist["epoch"].append(ep)
        hist["train_h"].append(th / max(n, 1))
        hist["train_q"].append(tq / max(n, 1))
        hist["val_h"].append(vh)
        hist["val_q"].append(vq)
        if vq < best[0]:
            best = (vq, _copy_params(net))
        if cfg.verbose and (ep % 10 == 0 or ep == cfg.epochs - 1):
            print("  epoch %3d  h %.4f/%.4f  q %.4f/%.4f (train/val)"
                  % (ep, th / max(n, 1), vh, tq / max(n, 1), vq))
    if best[1] is not None:
        _restore_params(net, best[1])
    hist["best_val_q"] = [best[0]]
    return hist


@dataclass
class PolicyTrainConfig:
    epochs: int = 150
    batch: int = 256
    lr: float = 2e-3
    weight_decay: float = 1e-4
    val_frac: float = 0.12
    near_weight: float = 2.0
    seed: int = 0
    verbose: bool = True


def train_policy(net, d: Dict[str, np.ndarray],
                 cfg: PolicyTrainConfig) -> Dict[str, list]:
    X, Y = d["X"], d["Y"]
    eps = np.unique(d["ep"])
    rng = np.random.default_rng(cfg.seed)
    rng.shuffle(eps)
    n_val = max(1, int(round(len(eps) * cfg.val_frac)))
    val = np.isin(d["ep"], eps[:n_val])
    tr = np.where(~val)[0]
    va = np.where(val)[0]
    net.set_norm(X[tr].mean(axis=0), X[tr].std(axis=0))
    opt = net.optimizer(cfg.lr, cfg.weight_decay)
    hist = {"train": [], "val": []}
    best = (float("inf"), None)
    if cfg.near_weight > 0:
        rmin = X[tr, :108].min(axis=1) * 8.0
        w = 1.0 + cfg.near_weight * np.exp(-rmin / 0.8)
    for ep in range(cfg.epochs):
        if cfg.near_weight > 0:
            idx = rng.choice(tr, size=len(tr), replace=True, p=w / w.sum())
        else:
            idx = rng.permutation(tr)
        tot = 0.0
        for s in range(0, len(idx), cfg.batch):
            sel = idx[s:s + cfg.batch]
            y, cache = net.forward(X[sel], cache=True)
            dlt = y - Y[sel]
            tot += float(np.sum(dlt ** 2))
            opt.zero_grad()
            net.backward(2.0 * dlt / dlt.size, cache)
            opt.step()
        v = net.forward(X[va]) - Y[va]
        vse = float(np.mean(v ** 2))
        hist["train"].append(tot / max(len(idx) * Y.shape[1], 1))
        hist["val"].append(vse)
        if vse < best[0]:
            best = (vse, [(p.copy()) for l in net.layers for p in l.params])
        if cfg.verbose and (ep % 20 == 0 or ep == cfg.epochs - 1):
            print("  epoch %3d  train %.5f  val %.5f" % (ep, hist["train"][-1], vse))
    if best[1] is not None:
        k = 0
        for l in net.layers:
            for p in l.params:
                p[...] = best[1][k]
                k += 1
    hist["best_val"] = [best[0]]
    return hist


def evaluate_predictions(net: SafetyNet,
                         d: Dict[str, np.ndarray]) -> Dict[str, float]:
    X = dataset_features(d)
    A = dataset_q_inputs(d)
    h = net.predict(X)
    q = net.predict_q(X, A) + np.asarray([lidar_lower_bound(
        r, a) for r, a in zip(dataset_ranges(d), d["action"])])
    yh, yq = d["h"], d["q"]
    out = {"n": float(len(h))}
    for name, p, y in (("h", h, yh), ("q", q, yq)):
        e = p - y
        out[name + "_mae"] = float(np.mean(np.abs(e)))
        out[name + "_rmse"] = float(np.sqrt(np.mean(e ** 2)))
        var = float(np.var(y))
        out[name + "_r2"] = float(1.0 - np.mean(e ** 2) / max(var, 1e-9))
    return out
