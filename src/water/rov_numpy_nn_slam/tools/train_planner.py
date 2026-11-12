#!/usr/bin/env python3
"""Train the local planner MLP: sonar sectors + goal -> 8 discrete directions."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import (goal_features, load_training_data,
                                 sector_features, split_by_episode)
from rov_numpy_nn_slam.nn.mlp import MLP

DIRECTION_NAMES = ["ahead", "ahead_left", "left", "back_left",
                   "back", "back_right", "right", "ahead_right"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
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
    sectors = sector_features(data["scans"], 16)
    goal = goal_features(data["pose"], data["goal"])
    X = np.concatenate([sectors, goal], axis=1).astype(np.float64)
    Y = np.asarray(data["actions"], dtype=np.int64)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = MLP([X.shape[1], args.hidden, args.hidden, 8],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    for epoch in range(1, args.epochs + 1):
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr[idx], Ytr[idx], loss="ce"))
        if epoch == 1 or epoch % 10 == 0 or epoch == args.epochs:
            acc = float(np.mean(np.argmax(net.predict(Xva), axis=1) == Yva))
            print("epoch %3d  train_ce %.4f  val_acc %.3f"
                  % (epoch, float(np.mean(losses)), acc))

    pred = np.argmax(net.predict(Xva), axis=1)
    accuracy = float(np.mean(pred == Yva))
    confusion = np.zeros((8, 8), dtype=np.int64)
    for t, p in zip(Yva, pred):
        confusion[int(t), int(p)] += 1
    per_class = {}
    for i in range(8):
        total = int(confusion[i].sum())
        per_class[DIRECTION_NAMES[i]] = (round(float(confusion[i, i]) / total, 3)
                                         if total else None)
    seconds = time.time() - t0
    metrics = {
        "model": "planner_mlp",
        "task": "discrete local action (8 directions) classification",
        "architecture": [int(X.shape[1]), args.hidden, args.hidden, 8],
        "input": "16 sonar sectors + goal polar (20)",
        "directions": DIRECTION_NAMES,
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "accuracy": round(accuracy, 4),
        "per_class_accuracy": per_class,
        "confusion": confusion.tolist(),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "planner_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_planner.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("planner accuracy %.3f (target >= 0.65) | %.1f s"
          % (accuracy, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
