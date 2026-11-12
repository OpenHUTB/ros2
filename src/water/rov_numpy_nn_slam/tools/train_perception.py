#!/usr/bin/env python3
"""Train the sonar denoising MLP: noisy 64-beam scan -> clean 64-beam scan."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from rov_numpy_nn_slam import config as C
from rov_numpy_nn_slam.datasets import (augment_scans, load_training_data,
                                 scan_features, split_by_episode)
from rov_numpy_nn_slam.nn.mlp import MLP


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--augment", type=int, default=2500,
                        help="synthetic random-pose scan pairs per scene")
    parser.add_argument("--no-augment", dest="augment", action="store_const",
                        const=0)
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
    X = scan_features(data["scans"]).astype(np.float64)
    Y = scan_features(data["scans_clean"]).astype(np.float64)
    residual = Y - X
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], residual[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    if args.augment > 0:
        aug_noisy, aug_clean = augment_scans(scenes, args.augment,
                                             seed=args.seed + 1)
        Xaug = scan_features(aug_noisy)
        Yaug = scan_features(aug_clean)
        Xtr = np.concatenate([Xtr, Xaug], axis=0)
        Ytr = np.concatenate([Ytr, Yaug - Xaug], axis=0)
        print("added %d synthetic scan pairs (%d per scene)"
              % (len(aug_noisy), args.augment))
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = MLP([C.SONAR_BEAMS, args.hidden, args.hidden, C.SONAR_BEAMS],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    history = []
    for epoch in range(1, args.epochs + 1):
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr[idx], Ytr[idx], loss="mse"))
        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
            pred = np.clip(Xva + net.predict(Xva), 0.0, 1.0)
            val_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
            noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
            print("epoch %3d  train_mse %.6f  val_rmse %.5f  noisy_rmse %.5f"
                  % (epoch, float(np.mean(losses)), val_rmse, noisy_rmse))
            history.append({"epoch": epoch,
                            "train_mse": round(float(np.mean(losses)), 6),
                            "val_rmse_norm": round(val_rmse, 5)})

    pred = np.clip(Xva + net.predict(Xva), 0.0, 1.0)
    span = C.SONAR_RANGE_MAX - C.SONAR_RANGE_MIN
    noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
    den_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    improvement = (1.0 - den_rmse / max(noisy_rmse, 1e-12)) * 100.0
    mae = float(np.mean(np.abs(pred - Yva)))
    # metrics restricted to beams that were not dropped out (valid measurements)
    valid = (Xva < 0.995) & (Xva > 0.02)
    noisy_rmse_valid = float(np.sqrt(np.mean((Xva[valid] - Yva[valid]) ** 2)))
    den_rmse_valid = float(np.sqrt(np.mean((pred[valid] - Yva[valid]) ** 2)))
    improvement_valid = (1.0 - den_rmse_valid / max(noisy_rmse_valid, 1e-12)) * 100.0
    seconds = time.time() - t0
    metrics = {
        "model": "perception_mlp",
        "task": "sonar denoising (64 beams), predicts residual clean-noisy",
        "target": "residual (clean - noisy), output = noisy + residual clipped",
        "architecture": [C.SONAR_BEAMS, args.hidden, args.hidden,
                         C.SONAR_BEAMS],
        "hidden": args.hidden,
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "augment_pairs": int(args.augment) * len(scenes),
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "rmse_noisy": round(noisy_rmse * span, 4),
        "rmse_denoised": round(den_rmse * span, 4),
        "rmse_noisy_norm": round(noisy_rmse, 5),
        "rmse_denoised_norm": round(den_rmse, 5),
        "rmse_improvement_pct": round(improvement, 2),
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
    print("denoise RMSE: noisy %.4f m -> denoised %.4f m (%.1f%% better)"
          % (metrics["rmse_noisy"], metrics["rmse_denoised"], improvement))
    print("MAE %.4f m  |  %.1f s" % (metrics["mae_m"], seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
