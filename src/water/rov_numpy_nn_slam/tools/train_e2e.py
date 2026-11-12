#!/usr/bin/env python3
"""Train the end-to-end CNN: camera image (48x64x3) -> (surge, yaw rate)."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import (image_to_input, load_training_data,
                                 split_by_episode)
from rov_numpy_nn_slam.nn.cnn import CNN


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--stride1", type=int, default=4,
                        help="stride of the first conv (48x64 -> 12x16 for 4)")
    parser.add_argument("--weight-yaw", type=float, default=2.0,
                        help="extra loss weight on the yaw-rate output")
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
    X = image_to_input(data["images"]).astype(np.float64)
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    Y = np.stack([np.clip(cmds[:, 0] / C.MAX_SURGE, -1.0, 1.0),
                  np.clip(cmds[:, 3] / C.MAX_YAW_RATE, -1.0, 1.0)], axis=1)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = CNN(3, C.CAM_HEIGHT, C.CAM_WIDTH, conv_channels=(8, 16), fc=64,
              out_dim=2, seed=args.seed, lr=args.lr, stride1=args.stride1)
    weights = np.array([1.0, float(args.weight_yaw)], dtype=np.float64)
    rng = np.random.default_rng(args.seed)
    best_vx = float("inf")
    best_state = None
    for epoch in range(1, args.epochs + 1):
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            xb, yb = Xtr[idx], Ytr[idx]
            pred = net.forward(xb)
            diff = pred - yb
            loss = float(np.mean(diff * diff * weights[None, :])
                         / weights.mean())
            grad = 2.0 * diff * weights[None, :] / (diff.size * weights.mean())
            net.backward(grad)
            net.step()
            losses.append(loss)
        if epoch == 1 or epoch % 2 == 0 or epoch == args.epochs:
            pred = net.predict(Xva)
            vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SURGE
            yaw_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_YAW_RATE
            if vx_mae < best_vx:
                best_vx = vx_mae
                best_state = [p.copy() for p in net.params()]
                print("  new best vx_MAE %.4f at epoch %d" % (vx_mae, epoch))
            print("epoch %2d  train_mse %.6f  val vx_MAE %.4f m/s  yaw_MAE %.4f rad/s"
                  % (epoch, float(np.mean(losses)), vx_mae, yaw_mae))

    if best_state is not None:
        for p, q in zip(net.params(), best_state):
            p[...] = q
    pred = net.predict(Xva)
    vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SURGE
    yaw_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_YAW_RATE
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    seconds = time.time() - t0
    metrics = {
        "model": "e2e_cnn",
        "task": "camera image (48x64x3) -> (vx, yaw_rate) behaviour cloning",
        "architecture": {"conv_channels": [8, 16], "fc": 64, "out_dim": 2,
                         "conv1_stride": args.stride1},
        "loss_weights": {"vx": 1.0, "yaw": float(args.weight_yaw)},
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
    net.save(os.path.join(args.models_dir, "e2e_cnn.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_e2e.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("e2e CNN: vx MAE %.4f m/s (target <= 0.25), yaw MAE %.4f rad/s | %.1f s"
          % (vx_mae, yaw_mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
