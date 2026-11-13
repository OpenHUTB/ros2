#!/usr/bin/env python3
"""Train the end-to-end CNN: camera image (48x64x3) -> (forward speed, yaw rate).

The image is built by the simulator from the forward lidar depth profile, so
the CNN only sees the geometric corridor ahead -- no goal or NFZ information.
Labels are the recorded expert commands expressed in the body frame.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from uav_numpy_nn_slam import config as C
from uav_numpy_nn_slam.datasets import (image_to_input, load_training_data,
                                 split_by_episode)
from uav_numpy_nn_slam.nn.cnn import CNN


def expert_body_speed(data) -> np.ndarray:
    """Convert the recorded world-frame expert command to (vf, w_yaw)."""
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    pose = np.asarray(data["pose"], dtype=np.float64)
    yaw = pose[:, 3]
    vf = cmds[:, 0] * np.cos(yaw) + cmds[:, 1] * np.sin(yaw)
    return np.stack([vf, cmds[:, 3]], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--no-flip", dest="flip", action="store_false",
                        help="disable the left/right mirror augmentation")
    parser.add_argument("--channels", type=int, default=8,
                        help="first conv channel count (second = 2x)")
    parser.add_argument("--fc", type=int, default=64)
    parser.add_argument("--stride1", type=int, default=4,
                        help="stride of the first conv (48x64 -> 12x16 for 4)")
    parser.add_argument("--weight-yaw", type=float, default=2.0,
                        help="extra loss weight on the yaw-rate output")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=50.0)
    parser.add_argument("--data-dir", default=C.RESULTS_DIR)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--force-data", action="store_true")
    parser.set_defaults(flip=True)
    args = parser.parse_args()

    t0 = time.time()
    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    data = load_training_data(args.data_dir, scenes=scenes,
                              episodes=args.episodes, seconds=args.seconds,
                              force=args.force_data)
    X = image_to_input(data["images"]).astype(np.float64)
    labels = expert_body_speed(data)
    Y = np.stack([np.clip(labels[:, 0] / C.MAX_VX, -1.0, 1.0),
                  np.clip(labels[:, 1] / C.MAX_YAW_RATE, -1.0, 1.0)], axis=1)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = CNN(3, C.CAM_HEIGHT, C.CAM_WIDTH,
              conv_channels=(int(args.channels), 2 * int(args.channels)),
              fc=int(args.fc), out_dim=2, seed=args.seed, lr=args.lr,
              stride1=args.stride1)
    net.opt.weight_decay = float(args.weight_decay)
    weights = np.array([1.0, float(args.weight_yaw)], dtype=np.float64)
    rng = np.random.default_rng(args.seed)
    best_score = float("inf")
    best_state = None
    best_metrics = None
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.60 * args.epochs))):
            net.set_lr(args.lr * 0.35)
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            xb, yb = Xtr[idx].copy(), Ytr[idx].copy()
            # left/right mirror augmentation: the forward speed is unchanged
            # while the yaw-rate target changes sign
            if args.flip:
                flip = rng.random(len(xb)) < 0.5
                if bool(flip.any()):
                    xb[flip] = xb[flip, :, :, ::-1]
                    yb[flip, 1] *= -1.0
            pred = net.forward(xb)
            diff = pred - yb
            loss = float(np.mean(diff * diff * weights[None, :])
                         / weights.mean())
            grad = 2.0 * diff * weights[None, :] / (diff.size * weights.mean())
            net.backward(grad)
            net.step()
            losses.append(loss)
        pred = net.predict(Xva)
        vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_VX
        yaw_mae = (float(np.mean(np.abs(pred[:, 1] - Yva[:, 1])))
                   * C.MAX_YAW_RATE)
        score = vx_mae / C.MAX_VX + 0.5 * yaw_mae / C.MAX_YAW_RATE
        if score < best_score:
            best_score = score
            best_state = [p.copy() for p in net.params()]
            best_metrics = (vx_mae, yaw_mae)
            print("  new best vx_MAE %.4f m/s at epoch %d (%.1f s)"
                  % (vx_mae, epoch, time.time() - t0))
        print("epoch %2d  train_loss %.6f  val vx_MAE %.4f m/s  yaw_MAE "
              "%.4f rad/s" % (epoch, float(np.mean(losses)), vx_mae, yaw_mae))

    if best_state is not None:
        for p, q in zip(net.params(), best_state):
            p[...] = q
    pred = net.predict(Xva)
    vx_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_VX
    yaw_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_YAW_RATE
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    # trivial baseline: always predict the mean label (report as reference)
    mean_label = Ytr.mean(axis=0, keepdims=True)
    base = net.predict(Xva) * 0.0 + mean_label
    base_vx_mae = float(np.mean(np.abs(base[:, 0] - Yva[:, 0]))) * C.MAX_VX
    seconds = time.time() - t0
    metrics = {
        "model": "e2e_cnn",
        "task": "camera image (48x64x3) -> (forward speed, yaw rate) "
                "behaviour cloning",
        "architecture": {"conv_channels": [int(args.channels),
                                           2 * int(args.channels)],
                         "fc": int(args.fc), "out_dim": 2,
                         "conv1_stride": args.stride1},
        "loss_weights": {"vx": 1.0, "yaw": float(args.weight_yaw)},
        "weight_decay": float(args.weight_decay),
        "augmentation": "left-right mirror (flip image, negate yaw target)",
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "vx_mae": round(vx_mae, 4),
        "yaw_mae": round(yaw_mae, 4),
        "mean_baseline_vx_mae": round(base_vx_mae, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "e2e_cnn.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_e2e.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("e2e CNN: vx MAE %.4f m/s (target <= 0.80), yaw MAE %.4f rad/s, "
          "mean-baseline vx MAE %.4f | %.1f s"
          % (vx_mae, yaw_mae, base_vx_mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
