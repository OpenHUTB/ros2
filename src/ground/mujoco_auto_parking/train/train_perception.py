#!/usr/bin/env python3
"""Train the parking-slot perception MLP from front-camera images."""

import csv
import math
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.nn_core import save_npz, train_mlp  # noqa: E402
from mujoco_auto_parking.parking_perception import (  # noqa: E402
    IMAGE_SIZES,
    TARGET_SLOT_XY,
    decode_output,
    encode_label,
    image_features,
    relative_slot_pose,
    wrap_angle,
)
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402


def is_free_pose(x: float, y: float) -> bool:
    obstacles = [(-3.5, 1.4), (3.5, -1.4)]
    for ox, oy in obstacles:
        if math.hypot(x - ox, y - oy) < 0.8:
            return False
    return True


def make_dataset(samples: int, seed: int = 42):
    rng = np.random.default_rng(seed)
    simulation = MujocoSimulation(PACKAGE_ROOT / 'models' / 'parking_lot.xml')
    camera = CameraSensor(
        simulation.model,
        simulation.data,
        camera_name='front_camera',
        width=160,
        height=120,
    )
    features = []
    labels = []
    target_x, target_y = TARGET_SLOT_XY
    attempts = 0
    while len(features) < samples and attempts < samples * 20:
        attempts += 1
        x = float(rng.uniform(-5.0, 4.5))
        y = float(rng.uniform(-1.5, 1.5))
        if not is_free_pose(x, y):
            continue
        bearing = math.atan2(target_y - y, target_x - x)
        yaw = bearing + float(rng.uniform(-0.6, 0.6))
        simulation.set_pose(x, y, yaw)
        rgb = camera.render_rgb()
        features.append(image_features(rgb))
        dx, dy, dtheta = relative_slot_pose(x, y, yaw)
        labels.append(encode_label(dx, dy, dtheta))
    return np.asarray(features), np.asarray(labels)


def evaluate(model, X, Y):
    predictions = model.forward(X)
    dx_errors, dy_errors, dt_errors = [], [], []
    for prediction, target in zip(predictions, Y):
        dx, dy, dtheta = decode_output(prediction)
        true_dx = float(target[0]) * 5.0
        true_dy = float(target[1]) * 5.0
        true_dtheta = float(np.arctan2(target[2], target[3]))
        dx_errors.append(abs(dx - true_dx))
        dy_errors.append(abs(dy - true_dy))
        dt_errors.append(abs(wrap_angle(dtheta - true_dtheta)))
    return {
        'mae_dx': float(np.mean(dx_errors)),
        'mae_dy': float(np.mean(dy_errors)),
        'mae_dtheta': float(np.mean(dt_errors)),
        'mse': float(np.mean((predictions - Y) ** 2)),
    }


def main() -> None:
    samples = 2000
    print(f'生成 {samples} 张前视相机样本...')
    X, Y = make_dataset(samples)
    split = int(0.8 * len(X))
    model, history = train_mlp(
        X[:split],
        Y[:split],
        IMAGE_SIZES,
        hidden_act='tanh',
        output_act='tanh',
        epochs=80,
        batch_size=128,
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
    weights_path = weights_dir / 'perception.npz'
    save_npz(weights_path, model)
    print(f'权值已保存: {weights_path}')

    with open(weights_dir / 'perception_training.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['epoch', 'train_loss', 'val_loss'])
        for index, (train_loss, val_loss) in enumerate(zip(
            history['train_loss'], history['val_loss']
        ), start=1):
            writer.writerow([index, train_loss, val_loss])
    print(f'训练日志已保存: {weights_dir / "perception_training.csv"}')


if __name__ == '__main__':
    main()
