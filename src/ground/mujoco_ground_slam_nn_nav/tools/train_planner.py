#!/usr/bin/env python3
"""Train the neural local planner (local map + goal -> 8-way direction)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, ROOT)

from gvnn.config import MODELS_DIR, RESULTS_DIR
from gvnn.datasets import (direction_label, goal_polar, local_patch,
                           sample_free_pose, random_goal)
from gvnn.geometry import resample_polyline
from gvnn.nn.mlp import train_model
from gvnn.nn.specs import planner_mlp
from gvnn.planning.astar import GridPlanner
from gvnn.sim.world import SCENES, occupancy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pairs-per-scene", type=int, default=45)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=8e-4)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    patches, goals, labels = [], [], []
    for name, scene in SCENES.items():
        occ = occupancy(scene)
        planner = GridPlanner(scene.width, scene.height, 0.10, 0.38, 1.0)
        planner.set_map(occ)
        made = 0
        attempts = 0
        while made < args.pairs_per_scene and attempts < args.pairs_per_scene * 4:
            attempts += 1
            start = sample_free_pose(scene, occ, rng, clearance=0.65)
            goal = random_goal(scene, occ, rng, start, min_distance=6.0)
            path = planner.plan(start, goal)
            if path is None:
                continue
            pts = resample_polyline(path, 0.35)
            for i, p in enumerate(pts[:-2]):
                nxt = pts[min(len(pts) - 1, i + 2)]
                yaw = math.atan2(nxt[1] - p[1], nxt[0] - p[0])
                pose = (p[0] + rng.normal(0, 0.05), p[1] + rng.normal(0, 0.05),
                        yaw + rng.normal(0, 0.06))
                patch = local_patch(occ, np.zeros_like(occ), pose, size=21, resolution=0.2)
                # Domain randomisation: unknown cells and sensor dropouts.
                if rng.random() < 0.7:
                    mask = rng.random(patch.shape) < 0.12
                    patch[mask] = 0.5
                patches.append(patch.reshape(-1))
                goals.append(goal_polar(pose, goal))
                labels.append(direction_label(pose, path, distance=2.5))
            made += 1
        print(f"{name}: {made} paths, {len(labels)} samples")
    X = np.column_stack([np.asarray(patches), np.asarray(goals)]).astype(np.float64)
    y = np.asarray(labels, dtype=np.int64)
    perm = rng.permutation(len(X))
    n_val = max(1, int(0.15 * len(X)))
    val, tr = perm[:n_val], perm[n_val:]
    model = planner_mlp(seed=args.seed)
    model.mean = X[tr].mean(axis=0)
    model.std = X[tr].std(axis=0) + 1e-6
    hist = train_model(model, X[tr], y[tr], epochs=args.epochs,
                       batch_size=args.batch, lr=args.lr, loss="ce",
                       val_fraction=0.0, seed=args.seed, verbose=True)
    pred = np.argmax(model.predict(X[val]), axis=1)
    acc = float(np.mean(pred == y[val]))
    metrics = {
        "train_loss": hist["train_loss"][-1],
        "val_accuracy": acc,
        "samples": int(len(X)),
        "epochs": args.epochs,
        "params": int(sum(v.size for v in model.params().values())),
    }
    os.makedirs(MODELS_DIR, exist_ok=True)
    path = os.path.join(MODELS_DIR, "planner_mlp.npz")
    model.save(path)
    with open(os.path.join(RESULTS_DIR, "train_planner.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print("saved", path, metrics)


if __name__ == "__main__":
    main()
