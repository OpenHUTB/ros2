#!/usr/bin/env python3
"""Train the end-to-end CNN: camera image (48x64x3) -> (throttle, steer).

The simulated camera shows the lane markings ahead, side obstacles, the
nearest traffic-light colour and pedestrian blobs, so the CNN can in principle
learn both the speed and the steering policy directly from pixels.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.datasets import (image_to_input, load_training_data,
                                  split_by_episode)
from road_numpy_nn_slam.nn.cnn import CNN


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--channels", type=int, default=8,
                        help="first conv channel count (second = 2x)")
    parser.add_argument("--fc", type=int, default=64)
    parser.add_argument("--stride1", type=int, default=4,
                        help="stride of the first conv (48x64 -> 12x16 for 4)")
    parser.add_argument("--weight-steer", type=float, default=1.0,
                        help="extra loss weight on the steering output")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--data-dir", default=C.RESULTS_DIR)
    parser.add_argument("--models-dir", default=C.MODELS_DIR)
    parser.add_argument("--force-data", action="store_true")
    parser.add_argument("--no-flip", dest="flip", action="store_false",
                        help="disable the left/right mirror augmentation")
    parser.set_defaults(flip=True)
    args = parser.parse_args()

    t0 = time.time()
    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    data = load_training_data(args.data_dir, scenes=scenes,
                              episodes=args.episodes, seconds=args.seconds,
                              force=args.force_data)
    X = image_to_input(data["images"]).astype(np.float64)
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    Y = np.stack([np.clip(cmds[:, 0], -1.0, 1.0),
                  np.clip(cmds[:, 1], -1.0, 1.0)], axis=1)
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
    weights = np.array([1.0, float(args.weight_steer)], dtype=np.float64)
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
            xb, yb = Xtr[idx].copy(), Ytr[idx].copy()
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
        thr_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0])))
        steer_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1])))
        score = thr_mae + steer_mae
        if score < best_score:
            best_score = score
            best_state = [p.copy() for p in net.params()]
            print("  new best throttle_MAE %.4f steer_MAE %.4f at epoch %d "
                  "(%.1f s)" % (thr_mae, steer_mae, epoch, time.time() - t0))
        print("epoch %2d  train_loss %.6f  val throttle_MAE %.4f  "
              "steer_MAE %.4f" % (epoch, float(np.mean(losses)), thr_mae,
                                  steer_mae))

    if best_state is not None:
        for p, q in zip(net.params(), best_state):
            p[...] = q
    pred = net.predict(Xva)
    thr_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0])))
    steer_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1])))
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    mean_label = Ytr.mean(axis=0, keepdims=True)
    base = pred * 0.0 + mean_label
    base_thr = float(np.mean(np.abs(base[:, 0] - Yva[:, 0])))
    base_steer = float(np.mean(np.abs(base[:, 1] - Yva[:, 1])))
    seconds = time.time() - t0
    metrics = {
        "model": "e2e_cnn",
        "task": "camera image (48x64x3) -> (throttle, steer) "
                "behaviour cloning",
        "architecture": {"conv_channels": [int(args.channels),
                                           2 * int(args.channels)],
                         "fc": int(args.fc), "out_dim": 2,
                         "conv1_stride": args.stride1},
        "loss_weights": {"throttle": 1.0, "steer": float(args.weight_steer)},
        "weight_decay": float(args.weight_decay),
        "augmentation": "left-right mirror (flip image, negate steer target)",
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "throttle_mae": round(thr_mae, 4),
        "steer_mae": round(steer_mae, 4),
        "mean_baseline_throttle_mae": round(base_thr, 4),
        "mean_baseline_steer_mae": round(base_steer, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "e2e_cnn.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_e2e.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("e2e CNN: throttle MAE %.4f (target <= 0.25), steer MAE %.4f "
          "(target <= 0.15), mean-baseline %.4f / %.4f | %.1f s"
          % (thr_mae, steer_mae, base_thr, base_steer, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
