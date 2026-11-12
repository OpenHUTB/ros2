#!/usr/bin/env python3
"""Behaviour-clone the reactive walking policy into the control MLP.

Input : 18 lidar sectors (min + mean = 36) + relative goal polar (3) +
        current walking speed (1) + lateral balance (1)  ->  41 dims
Output: (walk speed, turn rate) in the walker's physical units.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn import config as C
from humanoid_walk_nn.datasets import (augment_control, load_training_data,
                                       split_by_episode)
from humanoid_walk_nn.nn.mlp import MLP


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=96)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--hidden", type=int, default=160)
    parser.add_argument("--augment", type=int, default=2500,
                        help="synthetic reactive-expert states per scene")
    parser.add_argument("--no-augment", dest="augment", action="store_const",
                        const=0)
    parser.add_argument("--real-repeat", type=int, default=3,
                        help="oversample recorded expert states")
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--split", choices=("episode", "random"),
                        default="episode", help="validation split protocol")
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=90.0)
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
    if args.split == "random":
        rng_split = np.random.default_rng(args.seed + 99)
        perm = rng_split.permutation(len(X))
        n_val = max(1, int(round(len(X) * args.val_fraction)))
        val_mask = np.zeros(len(X), dtype=bool)
        val_mask[perm[:n_val]] = True
        train_mask = ~val_mask
        val_eps = ["random-holdout"]
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
        print("added %d synthetic reactive-expert samples (%d per scene)"
              % (len(Xaug), args.augment))
    print("labels: reactive expert; samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = MLP([X.shape[1], args.hidden, args.hidden, 2],
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
            pred = net.predict(Xva)
            v_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SPEED
            w_mae = (float(np.mean(np.abs(pred[:, 1] - Yva[:, 1])))
                     * C.MAX_TURN_RATE)
            print("epoch %3d  train_mse %.6f  val v_MAE %.4f m/s  "
                  "omega_MAE %.4f rad/s"
                  % (epoch, float(np.mean(losses)), v_mae, w_mae))

    pred = net.predict(Xva)
    v_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SPEED
    w_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_TURN_RATE
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    seconds = time.time() - t0
    metrics = {
        "model": "control_mlp",
        "task": "behaviour cloning of the reactive walking policy (v, omega)",
        "expert": "reactive_balance_limited",
        "architecture": [int(X.shape[1]), args.hidden, args.hidden, 2],
        "input": "18 lidar sectors (min+mean) + goal polar + speed + balance = 41",
        "input_dim": int(X.shape[1]),
        "output": "(v/MAX_SPEED, omega/MAX_TURN_RATE)",
        "augment_samples": int(args.augment) * len(scenes),
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "v_mae": round(v_mae, 4),
        "omega_mae": round(w_mae, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "control_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_control.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("control MLP: v MAE %.4f m/s (target <= 0.15), omega MAE %.4f "
          "rad/s (target <= 0.20) | %.1f s" % (v_mae, w_mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
