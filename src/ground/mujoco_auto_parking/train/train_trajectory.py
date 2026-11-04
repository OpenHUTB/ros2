#!/usr/bin/env python3
"""Train the neural trajectory controller from a pure-pursuit expert."""

import csv
import math
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_auto_parking.nn_core import save_npz, train_mlp  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    CONTROL_SIZES,
    TrajectoryController,
)


def make_dataset(samples: int, seed: int = 42):
    rng = np.random.default_rng(seed)
    features = []
    labels = []
    for _ in range(samples):
        ex = float(rng.uniform(-0.6, 0.6))
        ey = float(rng.uniform(-0.5, 0.5))
        epsi = float(rng.uniform(-1.0, 1.0))
        velocity = float(rng.uniform(0.0, 0.8))
        v_ref = float(rng.uniform(0.3, 0.6))
        feature = np.array([
            ex / 1.0,
            ey / 0.5,
            epsi / math.pi,
            velocity / 1.0,
            v_ref / 0.6,
        ], dtype=np.float64)
        speed, yaw_rate = TrajectoryController.expert_from_features(
            feature, v_ref
        )
        v_norm = 2.0 * (speed / v_ref) - 1.0
        wz_norm = yaw_rate / 1.2
        features.append(feature)
        labels.append(np.array([
            np.clip(v_norm, -1.0, 1.0),
            np.clip(wz_norm, -1.0, 1.0),
        ]))
    return np.asarray(features), np.asarray(labels)


def evaluate(model, X, Y):
    predictions = model.forward(X)
    v_errors = []
    wz_errors = []
    for prediction, target in zip(predictions, Y):
        v_errors.append(abs(float(prediction[0]) - float(target[0])))
        wz_errors.append(abs(float(prediction[1]) - float(target[1])))
    correlation_v = float(np.corrcoef(predictions[:, 0], Y[:, 0])[0, 1])
    correlation_wz = float(np.corrcoef(predictions[:, 1], Y[:, 1])[0, 1])
    return {
        'mae_v_norm': float(np.mean(v_errors)),
        'mae_wz_norm': float(np.mean(wz_errors)),
        'corr_v': correlation_v,
        'corr_wz': correlation_wz,
        'mse': float(np.mean((predictions - Y) ** 2)),
    }


def main() -> None:
    samples = 20000
    print(f'生成 {samples} 条轨迹控制样本...')
    X, Y = make_dataset(samples)
    split = int(0.8 * len(X))
    model, history = train_mlp(
        X[:split],
        Y[:split],
        CONTROL_SIZES,
        hidden_act='tanh',
        output_act='tanh',
        epochs=80,
        batch_size=256,
        lr=1e-3,
        val_split=0.2,
        seed=42,
        verbose=True,
    )
    metrics = evaluate(model, X[split:], Y[split:])
    print('验证集指标:')
    for key, value in metrics.items():
        print(f'  {key}: {value:.5f}')

    weights_dir = PACKAGE_ROOT / 'models'
    weights_dir.mkdir(parents=True, exist_ok=True)
    weights_path = weights_dir / 'trajectory.npz'
    save_npz(weights_path, model)
    print(f'权值已保存: {weights_path}')

    with open(weights_dir / 'trajectory_training.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['epoch', 'train_loss', 'val_loss'])
        for index, (train_loss, val_loss) in enumerate(zip(
            history['train_loss'], history['val_loss']
        ), start=1):
            writer.writerow([index, train_loss, val_loss])
    print(f'训练日志已保存: {weights_dir / "trajectory_training.csv"}')


if __name__ == '__main__':
    main()
