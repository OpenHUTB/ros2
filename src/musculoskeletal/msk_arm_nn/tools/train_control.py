#!/usr/bin/env python3
"""Behaviour-clone the analytic muscle-activation policy into the MLP.

Input : 28-dim arm/target/depth feature vector
Output: 4 muscle activations (shoulder flexor/extensor, elbow flexor/extensor)
Target: mean activation MAE <= 0.10.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn.datasets import (FEATURE_DIM, augment_control,
                                 load_training_data, split_by_episode)
from msk_arm_nn.nn.mlp import MLP

MUSCLE_NAMES = ["shoulder_flexor", "shoulder_extensor",
                "elbow_flexor", "elbow_extensor"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=96)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--hidden", type=int, default=160)
    parser.add_argument("--augment", type=int, default=2500,
                        help="synthetic expert states per scene")
    parser.add_argument("--no-augment", dest="augment", action="store_const",
                        const=0)
    parser.add_argument("--real-repeat", type=int, default=3)
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=25.0)
    parser.add_argument("--data-dir", default=C.RESULTS_DIR)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--force-data", action="store_true")
    args = parser.parse_args()

    t0 = time.time()
    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    data = load_training_data(args.data_dir, scenes=scenes,
                              episodes=args.episodes, seconds=args.seconds,
                              force=args.force_data)
    X = np.asarray(data["obs"], dtype=np.float64)
    Y = np.asarray(data["labels"], dtype=np.float64)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    if args.real_repeat > 1:
        Xtr = np.concatenate([Xtr] * int(args.real_repeat), axis=0)
        Ytr = np.concatenate([Ytr] * int(args.real_repeat), axis=0)
    if args.augment > 0:
        Xaug, Yaug, _act = augment_control(scenes, args.augment,
                                          seed=args.seed + 2)
        Xtr = np.concatenate([Xtr, Xaug], axis=0)
        Ytr = np.concatenate([Ytr, Yaug], axis=0)
        print("added %d synthetic expert states (%d per scene)"
              % (len(Xaug), args.augment))
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = MLP([int(X.shape[1]), args.hidden, args.hidden, 4],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.55 * args.epochs))):
            net.set_lr(args.lr * 0.35)
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr[idx], Ytr[idx], loss="mse"))
        if epoch == 1 or epoch % 10 == 0 or epoch == args.epochs:
            pred = np.clip(net.predict(Xva), 0.0, 1.0)
            mae = float(np.mean(np.abs(pred - Yva)))
            print("epoch %3d  train_mse %.6f  val activation_MAE %.4f"
                  % (epoch, float(np.mean(losses)), mae))

    pred = np.clip(net.predict(Xva), 0.0, 1.0)
    per_muscle = np.mean(np.abs(pred - Yva), axis=0)
    mae = float(per_muscle.mean())
    max_mae = float(per_muscle.max())
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    seconds = time.time() - t0
    metrics = {
        "model": "activation_mlp",
        "task": "behaviour cloning of the analytic muscle activation policy",
        "architecture": [int(X.shape[1]), args.hidden, args.hidden, 4],
        "input": "EE polar + target polar + q + dq + activation + 4x4 depth = 28",
        "input_dim": int(X.shape[1]),
        "output": "4 muscle activations in [0, 1]",
        "muscles": MUSCLE_NAMES,
        "augment_samples": int(args.augment) * len(scenes),
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "activation_mae": round(mae, 4),
        "per_muscle_mae": [round(float(v), 4) for v in per_muscle],
        "max_muscle_mae": round(max_mae, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "activation_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_control.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("activation MLP: MAE %.4f (target <= 0.10), per-muscle %s | %.1f s"
          % (mae, [round(float(v), 3) for v in per_muscle], seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
