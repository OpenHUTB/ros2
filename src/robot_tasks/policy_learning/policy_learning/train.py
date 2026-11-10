"""训练策略网络：MSE、Adam、早停、按验证误差恢复最优权重。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .dataset import Dataset
from .mlp import PolicyMLP

__all__ = ["TrainConfig", "metrics", "train"]


@dataclass
class TrainConfig:
    epochs: int = 120
    batch: int = 128
    lr: float = 1e-3
    weight_decay: float = 2e-4
    patience: int = 40
    min_delta: float = 1e-5
    seed: int = 0


def metrics(pred: np.ndarray, label: np.ndarray) -> Dict[str, float]:
    p = np.asarray(pred, np.float32).reshape(-1, 2)
    y = np.asarray(label, np.float32).reshape(-1, 2)
    err = p - y
    rmse = np.sqrt(np.mean(err * err, axis=0))
    return {"rmse_v": float(rmse[0]), "rmse_w": float(rmse[1]),
            "mse": float(np.mean(err * err))}


def _snapshot(net: PolicyMLP):
    return {k: getattr(net, k).copy()
            for k in ("W1", "b1", "W2", "b2", "W3", "b3")}


def _restore(net: PolicyMLP, snap) -> None:
    for k, v in snap.items():
        setattr(net, k, np.array(v, copy=True))


def train(tr: Dataset, va: Dataset, cfg: TrainConfig | None = None,
          verbose: bool = True) -> Tuple[PolicyMLP, Dict[str, List[float]]]:
    cfg = cfg or TrainConfig()
    net = PolicyMLP(n_in=int(tr.x.shape[1]), seed=cfg.seed)
    net.set_stats(tr.x, tr.y)

    rng = np.random.default_rng(cfg.seed)
    hist = {k: [] for k in ("epoch", "train_mse", "val_mse",
                            "val_rmse_v", "val_rmse_w")}
    best = float("inf")
    best_ep = -1
    snap = _snapshot(net)

    for ep in range(int(cfg.epochs)):
        order = rng.permutation(len(tr))
        total, nb = 0.0, 0
        for s in range(0, len(order), cfg.batch):
            sel = order[s:s + cfg.batch]
            total += net.train_batch(tr.x[sel], tr.y[sel], lr=cfg.lr,
                                     weight_decay=cfg.weight_decay)
            nb += 1
        tr_mse = float(np.mean((net.predict(tr.x) - tr.y) ** 2))
        va_met = metrics(net.predict(va.x), va.y)

        hist["epoch"].append(float(ep))
        hist["train_mse"].append(float(total / max(nb, 1)))
        hist["val_mse"].append(va_met["mse"])
        hist["val_rmse_v"].append(va_met["rmse_v"])
        hist["val_rmse_w"].append(va_met["rmse_w"])

        if va_met["mse"] < best - cfg.min_delta:
            best = va_met["mse"]
            best_ep = ep
            snap = _snapshot(net)
        elif ep - best_ep >= cfg.patience:
            if verbose:
                print("  验证误差 %d 轮未改善，早停于第 %d 轮" % (cfg.patience, ep))
            break

        if verbose and (ep % 10 == 0 or ep == cfg.epochs - 1):
            print("  epoch %3d  train=%.5f  val=%.5f  rmse(v,w)=(%.3f, %.3f)"
                  % (ep, tr_mse, va_met["mse"], va_met["rmse_v"],
                     va_met["rmse_w"]))

    _restore(net, snap)
    hist["best_epoch"] = best_ep
    hist["best_val_mse"] = best
    return net, hist
