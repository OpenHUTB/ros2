#!/usr/bin/env python3
"""Train the neural velocity/steering controller by distilling pure pursuit."""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import MODELS_DIR, RESULTS_DIR
from gvnn.nn.mlp import train_model
from gvnn.nn.specs import control_mlp


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=250)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()

    data = np.load(os.path.join(RESULTS_DIR, "expert_dataset.npz"))
    X = data["features"].astype(np.float64)
    cmd = data["commands"].astype(np.float64)
    episodes = data["episodes"]
    Y = np.column_stack([cmd[:, 0] / 1.1 * 2.0 - 1.0, cmd[:, 1] / 1.6])
    unique = np.unique(episodes)
    rng = np.random.default_rng(args.seed)
    rng.shuffle(unique)
    n_val = max(1, int(round(0.18 * len(unique))))
    val_eps = set(unique[:n_val].tolist())
    val = np.array([e in val_eps for e in episodes])
    tr = ~val
    model = control_mlp(seed=args.seed)
    model.mean = X[tr].mean(axis=0)
    model.std = X[tr].std(axis=0) + 1e-6
    hist = train_model(model, X[tr], Y[tr], epochs=args.epochs,
                       batch_size=args.batch, lr=args.lr, loss="mse",
                       val_fraction=0.0, seed=args.seed, verbose=True)
    pred = model.predict(X[val])
    v_pred = (pred[:, 0] + 1.0) * 0.5 * 1.1
    v_true = (Y[val, 0] + 1.0) * 0.5 * 1.1
    w_pred = pred[:, 1] * 1.6
    w_true = Y[val, 1] * 1.6
    metrics = {
        "train_loss": hist["train_loss"][-1],
        "val_v_mae": float(np.mean(np.abs(v_pred - v_true))),
        "val_w_mae": float(np.mean(np.abs(w_pred - w_true))),
        "epochs": args.epochs,
        "params": int(sum(v.size for v in model.params().values())),
    }
    os.makedirs(MODELS_DIR, exist_ok=True)
    path = os.path.join(MODELS_DIR, "control_mlp.npz")
    model.save(path)
    with open(os.path.join(RESULTS_DIR, "train_control.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print("saved", path, metrics)


if __name__ == "__main__":
    main()
