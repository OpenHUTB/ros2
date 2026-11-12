#!/usr/bin/env python3
"""Behaviour-clone the expert controller into the control MLP.

Input : 16 sonar sectors + relative goal polar features (20 dims)
Output: (surge m/s, yaw rate rad/s), normalised to [-1, 1] during training.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import (augment_control, goal_features,
                                 load_training_data, reactive_commands,
                                 sector_features, split_by_episode)
from rov_numpy_nn_slam.nn.mlp import MLP


def build_inputs(data) -> np.ndarray:
    sectors = sector_features(data["scans"], 16)
    goal = goal_features(data["pose"], data["goal"])
    return np.concatenate([sectors, goal], axis=1).astype(np.float64)


def recorded_labels(data) -> np.ndarray:
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    return np.stack([np.clip(cmds[:, 0] / C.MAX_SURGE, -1.0, 1.0),
                     np.clip(cmds[:, 3] / C.MAX_YAW_RATE, -1.0, 1.0)], axis=1)


def reactive_labels(data) -> np.ndarray:
    cmds = reactive_commands(data["pose"], data["goal"], data["scans"])
    return np.stack([np.clip(cmds[:, 0] / C.MAX_SURGE, -1.0, 1.0),
                     np.clip(cmds[:, 3] / C.MAX_YAW_RATE, -1.0, 1.0)], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1.5e-3)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--augment", type=int, default=1200,
                        help="synthetic reactive-expert states per scene")
    parser.add_argument("--no-augment", dest="augment", action="store_const",
                        const=0)
    parser.add_argument("--real-repeat", type=int, default=8,
                        help="oversample recorded expert states (distribution "
                             "of the held-out validation episodes)")
    parser.add_argument("--expert", choices=("reactive", "recorded"),
                        default="reactive",
                        help="expert used for the behaviour-cloning labels")
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--split", choices=("episode", "random"),
                        default="episode",
                        help="validation split protocol")
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--data-dir", default=C.RESULTS_DIR)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--force-data", action="store_true")
    args = parser.parse_args()

    t0 = time.time()
    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    data = load_training_data(args.data_dir, scenes=scenes,
                              episodes=args.episodes, seconds=args.seconds,
                              force=args.force_data)
    X = build_inputs(data)
    if args.expert == "reactive":
        Y = reactive_labels(data)
    else:
        Y = recorded_labels(data)
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
        Xaug, Yaug = augment_control(scenes, args.augment, seed=args.seed + 2)
        Xtr = np.concatenate([Xtr, Xaug], axis=0)
        Ytr = np.concatenate([Ytr, Yaug], axis=0)
        print("added %d synthetic reactive-expert samples (%d per scene)"
              % (len(Xaug), args.augment))
    print("labels: %s expert; samples: train %d / val %d (val episodes %s)"
          % (args.expert, len(Xtr), len(Xva), val_eps))

    net = MLP([X.shape[1], args.hidden, args.hidden, 2],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    for epoch in range(1, args.epochs + 1):
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr[idx], Ytr[idx], loss="mse"))
        if epoch == 1 or epoch % 10 == 0 or epoch == args.epochs:
            pred = net.predict(Xva)
            vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SURGE
            yaw_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_YAW_RATE
            print("epoch %3d  train_mse %.6f  val vx_MAE %.4f m/s  yaw_MAE %.4f rad/s"
                  % (epoch, float(np.mean(losses)), vx_mae, yaw_mae))

    pred = net.predict(Xva)
    vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SURGE
    yaw_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_YAW_RATE
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    seconds = time.time() - t0
    metrics = {
        "model": "control_mlp",
        "task": "behaviour cloning of the reactive vector-field expert "
                "(vx, yaw_rate)",
        "expert": args.expert,
        "architecture": [int(X.shape[1]), args.hidden, args.hidden, 2],
        "input": "16 sonar sectors + goal polar (20)",
        "augment_samples": int(args.augment) * len(scenes),
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "vx_mae": round(vx_mae, 4),
        "yaw_mae": round(yaw_mae, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "control_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_control.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("control MLP: vx MAE %.4f m/s (target <= 0.08), yaw MAE %.4f rad/s "
          "(target <= 0.12) | %.1f s"
          % (vx_mae, yaw_mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
