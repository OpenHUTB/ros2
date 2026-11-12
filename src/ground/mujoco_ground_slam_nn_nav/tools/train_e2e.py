#!/usr/bin/env python3
"""Train the end-to-end BEV-image -> (v, omega) CNN by behaviour cloning."""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import MODELS_DIR, RESULTS_DIR
from gvnn.nn.losses import mse
from gvnn.nn.mlp import train_model
from gvnn.nn.specs import e2e_cnn


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=35)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--flip", action="store_true", default=True)
    args = ap.parse_args()

    data = np.load(os.path.join(RESULTS_DIR, "expert_dataset.npz"))
    images = data["images"].astype(np.float64) / 255.0
    commands = data["commands"].astype(np.float64)
    episodes = data["episodes"]
    # Normalise targets into the tanh range.
    targets = np.column_stack([
        commands[:, 0] / 1.10 * 2.0 - 1.0,
        commands[:, 1] / 1.60,
    ])
    if args.flip:
        flipped = images[:, :, ::-1, :].copy()
        ft = targets.copy()
        ft[:, 1] *= -1.0
        images = np.concatenate([images, flipped], axis=0)
        targets = np.concatenate([targets, ft], axis=0)
        episodes = np.concatenate([episodes, episodes], axis=0)

    unique = np.unique(episodes)
    rng = np.random.default_rng(args.seed)
    rng.shuffle(unique)
    n_val = max(1, int(round(0.18 * len(unique))))
    val_eps = set(unique[:n_val].tolist())
    val_mask = np.array([e in val_eps for e in episodes])
    tr, va = ~val_mask, val_mask
    print(f"train={int(tr.sum())} val={int(va.sum())} episodes={len(unique)}")

    model = e2e_cnn(seed=args.seed)
    hist = train_model(model, images[tr], targets[tr], epochs=args.epochs,
                       batch_size=args.batch, lr=args.lr, loss="mse",
                       val_fraction=0.0, seed=args.seed, verbose=True)
    pred = model.predict(images[va])
    val_loss, _ = mse(targets[va], pred)
    v_mae = float(np.mean(np.abs((pred[:, 0] + 1.0) * 0.5 * 1.10 -
                                 (targets[va, 0] + 1.0) * 0.5 * 1.10)))
    w_mae = float(np.mean(np.abs(pred[:, 1] * 1.60 - targets[va, 1] * 1.60)))
    os.makedirs(MODELS_DIR, exist_ok=True)
    path = os.path.join(MODELS_DIR, "e2e_cnn.npz")
    model.save(path)
    metrics = {
        "train_loss": hist["train_loss"][-1],
        "val_loss": float(val_loss),
        "val_v_mae": v_mae,
        "val_w_mae": w_mae,
        "epochs": args.epochs,
        "train_frames": int(tr.sum()),
        "val_frames": int(va.sum()),
        "params": int(sum(v.size for v in model.params().values())),
    }
    with open(os.path.join(RESULTS_DIR, "train_e2e.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print("saved", path, metrics)


if __name__ == "__main__":
    main()
