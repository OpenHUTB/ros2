#!/usr/bin/env python3
"""Train the 11 -> 64 -> 64 -> 2 MLP by behaviour cloning from the PI expert.

Typical usage
-------------
python3 train_mlp.py \
  --csv data/flat.csv data/slope.csv data/low_friction.csv \
  --output models/mlp_policy.pt --norm-output models/norm_stats.npz

If no CSV is supplied, a small synthetic tracked-vehicle dataset is generated
so that the training pipeline can be smoke-tested without Gazebo.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (  # noqa: E402
    DEFAULT_HIDDEN,
    FEATURE_NAMES,
    INPUT_DIM,
    MLPController,
    NormStats,
    PIExpert,
)

SCENES = ("flat", "slope", "low_friction")


def _safe_float(value, default=0.0):
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def load_csvs(paths):
    rows = []
    for path in paths:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for raw in reader:
                mode = raw.get("mode", "")
                if mode and mode not in ("collect", "pi"):
                    continue
                row = {name: _safe_float(raw.get(name, 0.0)) for name in FEATURE_NAMES}
                row["v_cmd"] = _safe_float(raw.get("v_cmd", 0.0))
                row["w_cmd"] = _safe_float(raw.get("w_cmd", 0.0))
                row["scene"] = raw.get("scene", "flat") or "flat"
                rows.append(row)
    return rows


def rows_to_arrays(rows):
    features = np.asarray([[row[name] for name in FEATURE_NAMES] for row in rows],
                          dtype=np.float32)
    targets = np.asarray([[row["v_cmd"], row["w_cmd"]] for row in rows],
                         dtype=np.float32)
    scenes = np.asarray([row["scene"] for row in rows])
    return features, targets, scenes


def split_indices(n, val_ratio, seed, scenes, val_scenes=None):
    rng = np.random.RandomState(seed)
    if val_scenes:
        val_mask = np.isin(scenes, val_scenes)
        val_idx = np.where(val_mask)[0]
        train_idx = np.where(~val_mask)[0]
        if len(val_idx) == 0 or len(train_idx) == 0:
            raise ValueError("val-scenes 过滤后没有训练或验证样本")
        return train_idx, val_idx
    permutation = rng.permutation(n)
    val_size = max(1, int(n * val_ratio))
    return permutation[val_size:], permutation[:val_size]


def evaluate(model, features, targets, norm, target_scale, indices):
    model.eval()
    with torch.no_grad():
        x = torch.from_numpy(norm.normalize(features[indices]))
        pred = model(x).cpu().numpy()
    pred = np.clip(pred, -1.0, 1.0) * target_scale
    target = targets[indices]
    mae = np.mean(np.abs(pred - target), axis=0)
    rmse = np.sqrt(np.mean((pred - target) ** 2, axis=0))
    return pred, target, mae, rmse


def train(args):
    if args.csv:
        rows = load_csvs(args.csv)
    else:
        print("[train] 未提供 CSV，生成合成数据用于流程验证")
        rows = generate_synthetic()
    if len(rows) < 50:
        raise SystemExit("有效样本太少（{}），请先采集数据".format(len(rows)))

    features, targets, scenes = rows_to_arrays(rows)
    train_idx, val_idx = split_indices(
        len(features), args.val_ratio, args.seed, scenes,
        args.val_scenes if args.val_scenes else None,
    )
    norm = NormStats.from_features(features[train_idx])
    target_scale = np.asarray([args.max_v, args.max_w], dtype=np.float32)

    model = MLPController(hidden=args.hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr,
                                 weight_decay=args.weight_decay)
    loss_fn = nn.MSELoss()

    x_train = torch.from_numpy(norm.normalize(features[train_idx]))
    y_train = torch.from_numpy(
        np.clip(targets[train_idx] / target_scale, -1.0, 1.0)
    )
    x_val = torch.from_numpy(norm.normalize(features[val_idx]))
    y_val = torch.from_numpy(
        np.clip(targets[val_idx] / target_scale, -1.0, 1.0)
    )

    best_loss = float("inf")
    best_state = None
    patience = 0
    history = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        permutation = torch.randperm(len(x_train))
        total = 0.0
        for start in range(0, len(x_train), args.batch_size):
            idx = permutation[start:start + args.batch_size]
            optimizer.zero_grad()
            loss = loss_fn(model(x_train[idx]), y_train[idx])
            loss.backward()
            optimizer.step()
            total += float(loss.item()) * len(idx)
        train_loss = total / len(x_train)

        model.eval()
        with torch.no_grad():
            val_loss = float(loss_fn(model(x_val), y_val).item())
        history.append((epoch, train_loss, val_loss))
        if val_loss < best_loss - 1e-6:
            best_loss = val_loss
            best_state = {key: value.detach().clone()
                          for key, value in model.state_dict().items()}
            patience = 0
        else:
            patience += 1
        if epoch == 1 or epoch % 20 == 0 or patience == 0:
            print("[train] epoch {:4d} train={:.5f} val={:.5f}".format(
                epoch, train_loss, val_loss))
        if patience >= args.patience:
            print("[train] early stop at epoch", epoch)
            break

    if best_state is not None:
        model.load_state_dict(best_state)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.norm_output)), exist_ok=True)
    torch.save(model.state_dict(), args.output)
    norm.save(args.norm_output, target_scale=target_scale)

    config_path = os.path.join(os.path.dirname(os.path.abspath(args.output)),
                               "model_config.json")
    with open(config_path, "w", encoding="utf-8") as handle:
        json.dump({
            "input_dim": INPUT_DIM,
            "hidden": args.hidden,
            "output_dim": 2,
            "feature_names": FEATURE_NAMES,
            "target_scale": [float(args.max_v), float(args.max_w)],
            "training_samples": int(len(train_idx)),
            "validation_samples": int(len(val_idx)),
            "best_val_loss": float(best_loss),
            "epochs": len(history),
            "scenes": sorted(set(scenes.tolist())),
        }, handle, ensure_ascii=False, indent=2)

    pred, target, mae, rmse = evaluate(
        model, features, targets, norm, target_scale, val_idx
    )
    print("[train] 验证集 MAE  v={:.4f} m/s  w={:.4f} rad/s".format(mae[0], mae[1]))
    print("[train] 验证集 RMSE v={:.4f} m/s  w={:.4f} rad/s".format(rmse[0], rmse[1]))
    for scene in sorted(set(scenes[val_idx].tolist())):
        mask = scenes[val_idx] == scene
        scene_mae = np.mean(np.abs(pred[mask] - target[mask]), axis=0)
        print("[train]   {:13s} n={:4d} MAE v={:.4f} w={:.4f}".format(
            scene, int(mask.sum()), scene_mae[0], scene_mae[1]))

    print("[train] 权重: {}".format(args.output))
    print("[train] 归一化: {}".format(args.norm_output))
    return history


def generate_synthetic():
    """Generate a small, plausible dataset with a simple slip/slope model."""
    rng = np.random.RandomState(20261027)
    dt = 0.05
    rows = []
    expert = PIExpert(1.10, 1.60, 1.80, 3.20, 0.80, 1.00, 0.90, 1.60)
    for scene in SCENES:
        friction = {"flat": 0.96, "slope": 0.88, "low_friction": 0.55}[scene]
        pitch = 0.1745 if scene == "slope" else 0.0
        expert.reset()
        v_actual = 0.0
        w_actual = 0.0
        v_target = 0.0
        w_target = 0.0
        next_change = 0.0
        for step in range(int(45.0 / dt)):
            now = step * dt
            if now >= next_change:
                if scene == "slope":
                    v_target = rng.uniform(0.10, 0.65)
                    w_target = rng.uniform(-0.30, 0.30)
                else:
                    v_target = rng.uniform(-0.55, 0.80)
                    w_target = rng.uniform(-1.10, 1.10)
                    if abs(w_target) > 0.8:
                        v_target = np.clip(v_target, -0.25, 0.35)
                next_change = now + rng.uniform(2.0, 4.0)
            e_v = v_target - v_actual
            e_w = w_target - w_actual
            v_cmd, w_cmd = expert.command(v_target, w_target, v_actual, w_actual, dt)
            gravity = 9.81 * math.sin(pitch) * 0.35
            v_next = v_cmd * friction - gravity * 0.08
            v_actual += (v_next - v_actual) * min(1.0, dt / 0.35)
            w_actual += (w_cmd * (0.92 if scene == "low_friction" else 0.98)
                         - w_actual) * min(1.0, dt / 0.25)
            v_actual += rng.normal(0.0, 0.008)
            w_actual += rng.normal(0.0, 0.010)
            roll = 0.0 if scene != "slope" else rng.normal(0.0, 0.005)
            yaw_rate = w_actual + rng.normal(0.0, 0.005)
            row = dict(zip(FEATURE_NAMES, [
                v_target, w_target, v_actual, w_actual, e_v, e_w,
                expert.int_v, expert.int_w, pitch, roll, yaw_rate,
            ]))
            row["v_cmd"] = v_cmd
            row["w_cmd"] = w_cmd
            row["scene"] = scene
            rows.append(row)
    return rows


def parse_args():
    parser = argparse.ArgumentParser(description="Train the tracked vehicle MLP")
    parser.add_argument("--csv", nargs="*", default=[], help="采集的 CSV 文件")
    parser.add_argument("--output", default="models/mlp_policy.pt")
    parser.add_argument("--norm-output", default="models/norm_stats.npz")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--val-scenes", nargs="*", default=[],
                        help="留出某些场景做泛化验证，例如 --val-scenes slope")
    parser.add_argument("--patience", type=int, default=60)
    parser.add_argument("--seed", type=int, default=20261027)
    parser.add_argument("--max-v", type=float, default=0.90)
    parser.add_argument("--max-w", type=float, default=1.60)
    parser.add_argument("--hidden", nargs="+", type=int, default=DEFAULT_HIDDEN)
    return parser.parse_args()


def main():
    args = parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    torch.set_num_threads(1)
    history = train(args)
    if history:
        print("[train] 最终 epoch={} train={:.5f} val={:.5f}".format(*history[-1]))


if __name__ == "__main__":
    main()
