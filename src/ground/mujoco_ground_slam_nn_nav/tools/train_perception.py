#!/usr/bin/env python3
"""Train the lidar perception network (denoising / occupancy confidence)."""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import LIDAR_RANGE_MAX, MODELS_DIR, RESULTS_DIR
from gvnn.datasets import corrupt_scan, sample_free_pose
from gvnn.nn.mlp import train_model
from gvnn.nn.specs import perception_cnn
from gvnn.sim.mujoco_backend import MuJoCoSimulation
from gvnn.sim.world import SCENES, occupancy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples-per-scene", type=int, default=1200)
    ap.add_argument("--epochs", type=int, default=35)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    inputs, targets = [], []
    for name, scene in SCENES.items():
        occ = occupancy(scene)
        sim = MuJoCoSimulation(name, seed=int(rng.integers(1 << 30)), noise=False)
        for k in range(args.samples_per_scene):
            pose = sample_free_pose(scene, occ, rng, clearance=0.55)
            sim.reset(pose)
            clean = sim.scan(clean=True).astype(np.float64)
            noisy = corrupt_scan(clean, rng, severity=float(rng.uniform(0.5, 1.6)))
            inputs.append(noisy / LIDAR_RANGE_MAX)
            targets.append(clean / LIDAR_RANGE_MAX)
        print(f"{name}: generated {args.samples_per_scene}")
    X = np.asarray(inputs, dtype=np.float64).reshape(-1, 1, 360, 1)
    Y = np.asarray(targets, dtype=np.float64).reshape(-1, 1, 360, 1)
    perm = rng.permutation(len(X))
    n_val = max(1, int(0.15 * len(X)))
    val, tr = perm[:n_val], perm[n_val:]
    model = perception_cnn(seed=args.seed)
    residual = Y - X
    hist = train_model(model, X[tr], residual[tr], epochs=args.epochs,
                       batch_size=args.batch, lr=args.lr, loss="mse",
                       val_fraction=0.0, seed=args.seed, verbose=True)
    pred_res = model.predict(X[val])
    pred = np.clip(X[val] + pred_res, 0.0, 1.0).reshape(len(val), -1)
    Xf = X[val].reshape(len(val), -1)
    Yf = Y[val].reshape(len(val), -1)
    raw_rmse = float(np.sqrt(np.mean((Xf - Yf) ** 2)) * LIDAR_RANGE_MAX)
    filtered_rmse = float(np.sqrt(np.mean((pred - Yf) ** 2)) * LIDAR_RANGE_MAX)
    metrics = {
        "train_loss": hist["train_loss"][-1],
        "raw_rmse_m": raw_rmse,
        "filtered_rmse_m": filtered_rmse,
        "improvement_pct": (1.0 - filtered_rmse / max(1e-9, raw_rmse)) * 100.0,
        "epochs": args.epochs,
        "params": int(sum(v.size for v in model.params().values())),
    }
    os.makedirs(MODELS_DIR, exist_ok=True)
    path = os.path.join(MODELS_DIR, "perception_cnn.npz")
    model.save(path)
    with open(os.path.join(RESULTS_DIR, "train_perception.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print("saved", path, metrics)


if __name__ == "__main__":
    main()
