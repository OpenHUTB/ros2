#!/usr/bin/env python3
"""Train the end-to-end CNN: depth image (32x32) -> (walk speed, turn rate).

The simulated depth camera sees the corridor ahead but knows nothing about the
goal, so the network learns the local walking behaviour (slow down in front of
obstacles, turn away from walls) rather than the route.
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
from humanoid_walk_nn.datasets import (depth_to_input, load_training_data,
                                       split_by_episode)
from humanoid_walk_nn.nn.cnn import CNN


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=25)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--channels", type=int, default=8,
                        help="first conv channel count (second = 2x)")
    parser.add_argument("--fc", type=int, default=64)
    parser.add_argument("--stride1", type=int, default=2,
                        help="stride of the first conv (32x32 -> 16x16 for 2)")
    parser.add_argument("--weight-turn", type=float, default=1.0,
                        help="extra loss weight on the turn-rate output")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=90.0)
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
    X = depth_to_input(data["depth"]).astype(np.float64)
    cmds = np.asarray(data["cmds"], dtype=np.float64)
    Y = np.stack([np.clip(cmds[:, 0] / C.MAX_SPEED, -1.0, 1.0),
                  np.clip(cmds[:, 1] / C.MAX_TURN_RATE, -1.0, 1.0)], axis=1)
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = CNN(1, C.CAM_HEIGHT, C.CAM_WIDTH,
              conv_channels=(int(args.channels), 2 * int(args.channels)),
              fc=int(args.fc), out_dim=2, seed=args.seed, lr=args.lr,
              stride1=args.stride1)
    net.opt.weight_decay = float(args.weight_decay)
    weights = np.array([1.0, float(args.weight_turn)], dtype=np.float64)
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
        v_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SPEED
        w_mae = (float(np.mean(np.abs(pred[:, 1] - Yva[:, 1])))
                 * C.MAX_TURN_RATE)
        score = v_mae / C.MAX_SPEED + w_mae / C.MAX_TURN_RATE
        if score < best_score:
            best_score = score
            best_state = [p.copy() for p in net.params()]
            print("  new best v_MAE %.4f m/s omega_MAE %.4f rad/s at epoch %d "
                  "(%.1f s)" % (v_mae, w_mae, epoch, time.time() - t0))
        print("epoch %2d  train_loss %.6f  val v_MAE %.4f m/s  omega_MAE "
              "%.4f rad/s" % (epoch, float(np.mean(losses)), v_mae, w_mae))

    if best_state is not None:
        for p, q in zip(net.params(), best_state):
            p[...] = q
    pred = net.predict(Xva)
    v_mae = float(np.mean(np.abs(pred[:, 0] - Yva[:, 0]))) * C.MAX_SPEED
    w_mae = float(np.mean(np.abs(pred[:, 1] - Yva[:, 1]))) * C.MAX_TURN_RATE
    rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    mean_label = Ytr.mean(axis=0, keepdims=True)
    base = pred * 0.0 + mean_label
    base_v = float(np.mean(np.abs(base[:, 0] - Yva[:, 0]))) * C.MAX_SPEED
    base_w = float(np.mean(np.abs(base[:, 1] - Yva[:, 1]))) * C.MAX_TURN_RATE
    seconds = time.time() - t0
    metrics = {
        "model": "e2e_cnn",
        "task": "depth image (32x32) -> (walk speed, turn rate) "
                "behaviour cloning",
        "architecture": {"in_ch": 1, "in_h": C.CAM_HEIGHT,
                         "in_w": C.CAM_WIDTH,
                         "conv_channels": [int(args.channels),
                                           2 * int(args.channels)],
                         "fc": int(args.fc), "out_dim": 2,
                         "conv1_stride": args.stride1},
        "loss_weights": {"v": 1.0, "omega": float(args.weight_turn)},
        "weight_decay": float(args.weight_decay),
        "augmentation": "left-right mirror (flip depth image, negate turn)",
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "v_mae": round(v_mae, 4),
        "omega_mae": round(w_mae, 4),
        "mean_baseline_v_mae": round(base_v, 4),
        "mean_baseline_omega_mae": round(base_w, 4),
        "normalised_rmse": round(rmse, 5),
        "seconds": round(seconds, 2),
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "e2e_cnn.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_e2e.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("e2e CNN: v MAE %.4f m/s, omega MAE %.4f rad/s, mean-baseline "
          "%.4f / %.4f | %.1f s" % (v_mae, w_mae, base_v, base_w, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
