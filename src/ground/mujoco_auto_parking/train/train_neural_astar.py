#!/usr/bin/env python3
"""Train the Neural A* guidance network on random occupancy grids."""

import csv
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_auto_parking.neural_path_planner import (  # noqa: E402
    GUIDANCE_FEATURE_DIM,
    BaselineAStarPlanner,
    analytic_guidance,
    guidance_features,
)
from mujoco_auto_parking.nn_core import save_npz, train_mlp  # noqa: E402

GRID_SIZE = 60
SAMPLES_PER_GRID = 600


def random_problem(rng: np.random.Generator):
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int8)
    for _ in range(int(rng.integers(5, 11))):
        x0 = int(rng.integers(0, GRID_SIZE - 10))
        y0 = int(rng.integers(0, GRID_SIZE - 10))
        width = int(rng.integers(3, 11))
        height = int(rng.integers(3, 11))
        grid[y0:y0 + height, x0:x0 + width] = 100
    planner = BaselineAStarPlanner(resolution=1.0, clearance_m=1.0)
    inflated = planner._inflate_obstacles(grid, 1)
    free = np.argwhere(~inflated)
    if len(free) < 200:
        return None
    start_index, goal_index = rng.choice(len(free), size=2, replace=False)
    sy, sx = free[start_index]
    gy, gx = free[goal_index]
    if np.hypot(gx - sx, gy - sy) < 15:
        return None
    return grid, inflated, int(sx), int(sy), int(gx), int(gy)


def make_dataset(grids: int = 200, seed: int = 42):
    rng = np.random.default_rng(seed)
    features = []
    labels = []
    created = 0
    attempts = 0
    while created < grids and attempts < grids * 20:
        attempts += 1
        problem = random_problem(rng)
        if problem is None:
            continue
        grid, inflated, sx, sy, gx, gy = problem
        all_features = guidance_features(
            GRID_SIZE, GRID_SIZE, sx, sy, gx, gy, inflated
        )
        all_labels = analytic_guidance(
            GRID_SIZE, GRID_SIZE, sx, sy, gx, gy, inflated
        ).reshape(-1)
        valid_indices = np.flatnonzero(~inflated.reshape(-1))
        if len(valid_indices) == 0:
            continue
        chosen = rng.choice(
            valid_indices,
            size=min(SAMPLES_PER_GRID, len(valid_indices)),
            replace=False,
        )
        features.append(all_features[chosen])
        labels.append(all_labels[chosen])
        created += 1
    return np.vstack(features), np.concatenate(labels).reshape(-1, 1)


def main() -> None:
    print('生成随机栅格上的 Neural A* 训练样本...')
    X, Y = make_dataset()
    split = int(0.8 * len(X))
    model, history = train_mlp(
        X[:split],
        Y[:split],
        (GUIDANCE_FEATURE_DIM, 64, 32, 1),
        hidden_act='tanh',
        output_act='sigmoid',
        epochs=60,
        batch_size=512,
        lr=1e-3,
        val_split=0.2,
        seed=42,
        verbose=True,
    )
    validation = model.forward(X[split:])
    correlation = float(np.corrcoef(
        validation.reshape(-1), Y[split:].reshape(-1)
    )[0, 1])
    mse = float(np.mean((validation - Y[split:]) ** 2))
    print(f'验证 MSE={mse:.6f}, 引导场相关系数={correlation:.5f}')

    weights_dir = PACKAGE_ROOT / 'models'
    weights_dir.mkdir(parents=True, exist_ok=True)
    weights_path = weights_dir / 'planner.npz'
    save_npz(weights_path, model)
    print(f'权值已保存: {weights_path}')

    with open(weights_dir / 'planner_training.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['epoch', 'train_loss', 'val_loss'])
        for index, (train_loss, val_loss) in enumerate(zip(
            history['train_loss'], history['val_loss']
        ), start=1):
            writer.writerow([index, train_loss, val_loss])


if __name__ == '__main__':
    main()
