#!/usr/bin/env python3
"""Train the depth-image denoising MLP for the musculoskeletal arm.

The 32x32 depth camera of the arm is noisy (additive Gaussian sensor noise);
the perception node publishes a filtered image on ``/msk/depth_filtered``.
The network predicts the residual ``clean - noisy`` for the 1024 depth pixels
and the filtered image is ``clip(noisy + residual, 0, 1)``.
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
from msk_arm_nn.datasets import load_training_data, split_by_episode
from msk_arm_nn.nn.mlp import MLP

PIXELS = C.CAM_HEIGHT * C.CAM_WIDTH       # 1024


def flatten_depth(images: np.ndarray) -> np.ndarray:
    img = np.asarray(images)
    if img.ndim == 4:
        img = img[..., 0] if img.shape[-1] == 1 else img
    img = img.reshape(img.shape[0], -1).astype(np.float64)
    if float(img.max()) > 1.5:
        img = img / 255.0
    return np.clip(img, 0.0, 1.0)


def box3(flat: np.ndarray) -> np.ndarray:
    """3x3 mean filter of the flattened 32x32 depth images (edge padded)."""
    img = flat.reshape(-1, C.CAM_HEIGHT, C.CAM_WIDTH)
    padded = np.pad(img, ((0, 0), (1, 1), (1, 1)), mode="edge")
    acc = np.zeros_like(img)
    for dr in range(3):
        for dc in range(3):
            acc += padded[:, dr:dr + C.CAM_HEIGHT, dc:dc + C.CAM_WIDTH]
    return (acc / 9.0).reshape(flat.shape)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1.5e-3)
    parser.add_argument("--hidden", type=int, default=256)
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
                              force=args.force_data,
                              keys=["depth", "depth_clean", "episode"])
    X = flatten_depth(data["depth"])            # noisy
    Y = flatten_depth(data["depth_clean"])      # clean target
    train_mask, val_mask, val_eps = split_by_episode(
        data["episode"], args.val_fraction, args.seed)
    Xtr, Ytr = X[train_mask], Y[train_mask]
    Xva, Yva = X[val_mask], Y[val_mask]
    # the MLP corrects a classical 3x3 mean-filtered image: the network adds
    # edge-preserving detail on top of a smoother (a plain FC map cannot
    # represent a local image filter on this data)
    Str, Sva = box3(Xtr), box3(Xva)
    Xtr_in = np.concatenate([Xtr, Str], axis=1)
    Xva_in = np.concatenate([Xva, Sva], axis=1)
    residual = Ytr - Str
    print("samples: train %d / val %d (val episodes %s)"
          % (len(Xtr), len(Xva), val_eps))

    net = MLP([Xtr_in.shape[1], args.hidden, args.hidden, PIXELS],
              activation="relu", seed=args.seed, lr=args.lr)
    rng = np.random.default_rng(args.seed)
    history = []
    for epoch in range(1, args.epochs + 1):
        if epoch == max(2, int(round(0.60 * args.epochs))):
            net.set_lr(args.lr * 0.40)
        order = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(order), args.batch):
            idx = order[start:start + args.batch]
            losses.append(net.train_step(Xtr_in[idx], residual[idx], loss="mse"))
        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
            pred = np.clip(Sva + net.predict(Xva_in), 0.0, 1.0)
            val_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
            noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
            print("epoch %3d  train_mse %.6f  val_rmse %.5f  noisy_rmse %.5f"
                  % (epoch, float(np.mean(losses)), val_rmse, noisy_rmse))
            history.append({"epoch": epoch,
                            "train_mse": round(float(np.mean(losses)), 6),
                            "val_rmse": round(val_rmse, 5),
                            "noisy_rmse": round(noisy_rmse, 5)})

    pred = np.clip(Sva + net.predict(Xva_in), 0.0, 1.0)
    noisy_rmse = float(np.sqrt(np.mean((Xva - Yva) ** 2)))
    den_rmse = float(np.sqrt(np.mean((pred - Yva) ** 2)))
    improvement = (1.0 - den_rmse / max(noisy_rmse, 1e-12)) * 100.0
    smooth_rmse = float(np.sqrt(np.mean((Sva - Yva) ** 2)))
    smooth_improvement = (1.0 - smooth_rmse / max(noisy_rmse, 1e-12)) * 100.0
    mae = float(np.mean(np.abs(pred - Yva)))
    # denoising quality on the flat background (the physically relevant part)
    flat = (Yva < 0.99) & (Yva > 0.05)
    noisy_flat = float(np.sqrt(np.mean((Xva[flat] - Yva[flat]) ** 2)))
    den_flat = float(np.sqrt(np.mean((pred[flat] - Yva[flat]) ** 2)))
    flat_improvement = (1.0 - den_flat / max(noisy_flat, 1e-12)) * 100.0
    seconds = time.time() - t0
    metrics = {
        "model": "perception_mlp",
        "task": "32x32 depth-image denoising, predicts residual clean-noisy",
        "target": "residual (clean - noisy), output = noisy + residual clipped",
        "architecture": [int(Xtr_in.shape[1]), args.hidden, args.hidden,
                         PIXELS],
        "context": "3x3 mean-filtered depth context, MLP predicts the "
                   "residual from the smoothed image",
        "pixels": PIXELS,
        "hidden": args.hidden,
        "epochs": args.epochs,
        "batch": args.batch,
        "lr": args.lr,
        "train_samples": int(len(Xtr)),
        "val_samples": int(len(Xva)),
        "val_episodes": val_eps,
        "rmse_noisy": round(noisy_rmse, 5),
        "rmse_denoised": round(den_rmse, 5),
        "rmse_improvement_pct": round(improvement, 2),
        "rmse_noisy_flat": round(noisy_flat, 5),
        "rmse_denoised_flat": round(den_flat, 5),
        "rmse_improvement_flat_pct": round(flat_improvement, 2),
        "mae": round(mae, 5),
        "rmse_smoothed": round(smooth_rmse, 5),
        "rmse_smoothed_improvement_pct": round(smooth_improvement, 2),
        "seconds": round(seconds, 2),
        "history": history,
    }
    os.makedirs(args.models_dir, exist_ok=True)
    net.save(os.path.join(args.models_dir, "perception_mlp.npz"), extra=metrics)
    with open(os.path.join(C.RESULTS_DIR, "train_perception.json"), "w",
              encoding="utf-8") as fh:
        json.dump(metrics, fh, indent=2, sort_keys=True)
    print("depth denoise RMSE: noisy %.4f -> denoised %.4f (%.1f%% better)"
          % (noisy_rmse, den_rmse, improvement))
    print("MAE %.4f  |  %.1f s" % (mae, seconds))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
