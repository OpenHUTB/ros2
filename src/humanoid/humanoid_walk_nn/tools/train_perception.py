#!/usr/bin/env python3
"""Train the lidar denoising MLP: noisy 72-beam scan -> clean 72-beam scan."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from humanoid_walk_nn import config as C
from humanoid_walk_nn.datasets import (augment_scans, load_training_data,
                                       scan_features, split_by_episode)
from humanoid_walk_nn.nn.mlp import MLP


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=45)
    parser.add_argument("--batch", type=int, default=96)
    parser.add_argument("--lr", type=float, default=1.5e-3)
    parser.add_argument("--hidden", type=int, default=160)
    parser.add_argument("--augment", type=int, default=2500,
                        help="synthetic random-pose scan pairs per scene")
    parser.add_argument("--no-augment", dest="augment", action="store_const",
                        const=0)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--val-fraction", type=float, default=0.25)
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
    def median_context(scans_norm: np.ndarray) -> np.ndarray:
        """Circular median-5 of the normalised scan (lidar is 360 degrees)."""
        return np.median(np.stack([np.roll(scans_norm, o, axis=1)
                                   for o in range(-2, 3)], axis=0), axis=0)

    X = scan_features(data["scans"]).astype(np.float64)
    Y = scan_features(data["scans_clean"]).astype(np.float64)
    M = median_context(X)
    # the MLP predicts the correction from the median-filtered scan to the
    # clean scan; the circular median gives the network explicit local
    # neighbourhood context (a plain fully-connected scan-to-scan map cannot
    # represent a local robust filter on this data)
    residual = Y - M
    X_in = np.concatenate([X, M], axis=1)
    median_rmse_norm = float(np.sqrt(np.mean((M - Y) ** 2)))
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], residual[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    Mva = median_context(Xva)
    Xtr_in = np.concatenate([Xtr, median_context(Xtr)], axis=1)
    Xva_in = np.concatenate([Xva, median_context(Xva)], axis=1)
    if args.augment > 0:
        aug_noisy, aug_clean = augment_scans(scenes, args.augment,
                                             seed=args.seed + 1)
        Xaug = scan_features(aug_noisy)
        Yaug = scan_features(aug_clean)
        Maug = median_context(Xaug)
        Xtr_in = np.concatenate([Xtr_in,
                                 np.concatenate([Xaug, Maug], axis=1)], axis=0)
        Ytr = np.concatenate([Ytr, Yaug - Maug], axis=0)
        print("added %d synthetic scan pairs (%d per scene)"
              % (len(aug_noisy), args.augment))
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    beams = int(C.LIDAR_BEAMS)
    net = MLP([Xtr_in.shape[1], args.hidden, args.hidden, beams],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    history = []
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.60 * args.epochs))):
            net.set_lr(args.lr * 0.40)
        order = rng.permutation(len(Xtr_in))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr_in[idx], Ytr[idx], loss="mse"))
        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
            pred = np.clip(Mva + net.predict(Xva_in), 0.0, 1.0)
            val_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
            noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
            print("epoch %3d  train_mse %.6f  val_rmse %.5f  noisy_rmse %.5f"
                  % (epoch, float(np.mean(losses)), val_rmse, noisy_rmse))
            history.append({"epoch": epoch,
                            "train_mse": round(float(np.mean(losses)), 6),
                            "val_rmse_norm": round(val_rmse, 5),
                            "noisy_rmse_norm": round(noisy_rmse, 5)})

    pred = np.clip(Mva + net.predict(Xva_in), 0.0, 1.0)
    span = C.LIDAR_RANGE_MAX - C.LIDAR_RANGE_MIN
    noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
    den_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    improvement = (1.0 - den_rmse / max(noisy_rmse, 1e-12)) * 100.0
    mae = float(np.mean(np.abs(pred - Yva)))
    valid = (Xva < 0.995) & (Xva > 0.02)
    noisy_rmse_valid = float(np.sqrt(np.mean((Xva[valid] - Yva[valid]) ** 2)))
    den_rmse_valid = float(np.sqrt(np.mean((pred[valid] - Yva[valid]) ** 2)))
    improvement_valid = (1.0 - den_rmse_valid
                         / max(noisy_rmse_valid, 1e-12)) * 100.0
    seconds = time.time() - t0
    metrics = {
        "model": "perception_mlp",
        "task": "lidar denoising (72 beams), predicts residual clean-noisy",
        "target": "residual (clean - noisy), output = noisy + residual clipped",
        "architecture": [int(Xtr_in.shape[1]), args.hidden, args.hidden, beams],
        "beams": beams,
        "hidden": args.hidden,
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "augment_pairs": int(args.augment) * len(scenes),
        "train_samples": int(len(Xtr_in)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "rmse_noisy": round(noisy_rmse * span, 4),
        "rmse_denoised": round(den_rmse * span, 4),
        "rmse_noisy_norm": round(noisy_rmse, 5),
        "rmse_denoised_norm": round(den_rmse, 5),
        "rmse_improvement_pct": round(improvement, 2),
        "median_baseline_rmse": round(median_rmse_norm * span, 4),
        "median_baseline_improvement_pct": round(
            (1.0 - median_rmse_norm / max(noisy_rmse, 1e-12)) * 100.0, 2),
        "context": "circular median-5 lidar context, MLP predicts the "
                   "correction from the median-filtered scan",
        "rmse_noisy_valid": round(noisy_rmse_valid * span, 4),
        "rmse_denoised_valid": round(den_rmse_valid * span, 4),
        "rmse_improvement_valid_pct": round(improvement_valid, 2),
        "mae_norm": round(mae, 5),
        "mae_m": round(mae * span, 4),
        "seconds": round(seconds, 2),
        "history": history,
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "perception_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_perception.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("denoise RMSE: noisy %.4f m -> denoised %.4f m (%.1f%% better, "
          "target >= 20%%)" % (metrics["rmse_noisy"],
                               metrics["rmse_denoised"], improvement))
    print("MAE %.4f m  |  %.1f s" % (metrics["mae_m"], seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
