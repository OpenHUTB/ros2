#!/usr/bin/env python3
"""Train the end-to-end CNN: depth image (32x32) -> 4 muscle activations.

The depth camera sees the workspace in front of the arm base; the network
learns the local muscle co-activation pattern (and thus the reaching motion)
directly from pixels.
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
from msk_arm_nn.datasets import (depth_to_input, load_training_data,
                                 split_by_episode)
from msk_arm_nn.nn.cnn import CNN

MUSCLE_NAMES = ["shoulder_flexor", "shoulder_extensor",
                "elbow_flexor", "elbow_extensor"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=22)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--channels", type=int, default=8)
    parser.add_argument("--fc", type=int, default=64)
    parser.add_argument("--stride1", type=int, default=2)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
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
    X = depth_to_input(data["depth"]).astype(np.float64)
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    Y = np.clip(cmds[:, :4], 0.0, 1.0)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = CNN(1, C.CAM_HEIGHT, C.CAM_WIDTH,
              conv_channels=(int(args.channels), 2 * int(args.channels)),
              fc=int(args.fc), out_dim=4, seed=args.seed, lr=args.lr,
              stride1=args.stride1)
    net.opt.weight_decay = float(args.weight_decay)
    rng = np.random.default_rng(args.seed)
    best_score = float("inf")
    best_state = None
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.60 * args.epochs))):
            net.set_lr(args.lr * 0.35)
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            pred = net.forward(Xtr[idx])
            diff = pred - Ytr[idx]
            loss = float(np.mean(diff * diff))
            net.backward(2.0 * diff / diff.size)
            net.step()
            losses.append(loss)
        pred = np.clip(net.predict(Xva), 0.0, 1.0)
        mae = float(np.mean(np.abs(pred - Yva)))
        if mae < best_score:
            best_score = mae
            best_state = [p.copy() for p in net.params()]
            print("  new best activation_MAE %.4f at epoch %d (%.1f s)"
                  % (mae, epoch, time.time() - t0))
        print("epoch %2d  train_loss %.6f  val activation_MAE %.4f"
              % (epoch, float(np.mean(losses)), mae))

    if best_state is not None:
        for p, q in zip(net.params(), best_state):
            p[...] = q
    pred = np.clip(net.predict(Xva), 0.0, 1.0)
    per_muscle = np.mean(np.abs(pred - Yva), axis=0)
    mae = float(per_muscle.mean())
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    base = np.clip(Ytr.mean(axis=0, keepdims=True)
                   + pred * 0.0, 0.0, 1.0)
    base_mae = float(np.mean(np.abs(base - Yva)))
    seconds = time.time() - t0
    metrics = {
        "model": "e2e_cnn",
        "task": "depth image (32x32) -> 4 muscle activations",
        "architecture": {"in_ch": 1, "in_h": C.CAM_HEIGHT,
                         "in_w": C.CAM_WIDTH,
                         "conv_channels": [int(args.channels),
                                           2 * int(args.channels)],
                         "fc": int(args.fc), "out_dim": 4,
                         "conv1_stride": args.stride1},
        "muscles": MUSCLE_NAMES,
        "weight_decay": float(args.weight_decay),
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "activation_mae": round(mae, 4),
        "per_muscle_mae": [round(float(v), 4) for v in per_muscle],
        "mean_baseline_activation_mae": round(base_mae, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "e2e_cnn.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_e2e.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("e2e CNN: activation MAE %.4f (mean baseline %.4f), per-muscle %s "
          "| %.1f s" % (mae, base_mae,
                        [round(float(v), 3) for v in per_muscle], seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
