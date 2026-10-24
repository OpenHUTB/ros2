#!/usr/bin/env python3
"""Collect expert S-curve drives and train the end-to-end image-to-control CNN."""

import math
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.end_to_end_networks import (  # noqa: E402
    IMG_SIZE,
    NOMINAL_SPEED,
    WZ_SCALE,
    rgb_to_gray_resize,
    train_network,
)
from mujoco_auto_parking.kinematics import DriveGeometry  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    ReferenceTrajectory,
    TrajectoryController,
    build_s_curve,
)


def collect_episode(
    simulation,
    camera,
    controller,
    geometry,
    start_x,
    start_y,
    start_yaw,
    max_time=14.0,
):
    simulation.reset()
    simulation.set_pose(start_x, start_y, start_yaw)
    trajectory = ReferenceTrajectory(
        build_s_curve(start_x, start_y, start_yaw)
    )
    images = []
    labels = []
    control_period = 0.02
    steps = max(1, int(round(control_period / simulation.timestep)))
    time_elapsed = 0.0
    counter = 0
    while time_elapsed < max_time:
        state = simulation.planar_state()
        index, distance, reference = trajectory.lookahead(state.x, state.y)
        speed, yaw_rate = controller.predict(
            (state.x, state.y, state.yaw, state.linear_velocity),
            reference,
            0.45,
        )
        if counter % 2 == 0:
            rgb = camera.render_rgb()
            images.append(rgb_to_gray_resize(rgb, IMG_SIZE))
            labels.append([
                float(np.clip(2.0 * speed / NOMINAL_SPEED - 1.0, -1.0, 1.0)),
                float(np.clip(yaw_rate / WZ_SCALE, -1.0, 1.0)),
            ])
        left, right = geometry.wheel_speeds(speed, yaw_rate)
        for _ in range(steps):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        time_elapsed += control_period
        counter += 1
        if index >= len(trajectory.points) - 4 and distance < 0.25:
            break
    return images, labels


def main() -> None:
    simulation = MujocoSimulation(PACKAGE_ROOT / 'models' / 'parking_lot.xml')
    camera = CameraSensor(
        simulation.model, simulation.data, width=160, height=120
    )
    controller = TrajectoryController(
        PACKAGE_ROOT / 'models' / 'trajectory.npz'
    )
    geometry = DriveGeometry(0.08, 0.48, 8.0)

    rng = np.random.default_rng(7)
    X = []
    Y = []
    starts = [
        (-6.0, 0.0, 0.0),
        (-6.0, 0.4, 0.1),
        (-6.0, -0.4, -0.1),
        (-5.5, 0.2, 0.2),
        (-6.2, -0.2, -0.2),
    ]
    for start_x, start_y, start_yaw in starts:
        print(f'采集专家轨迹: start=({start_x:.1f}, {start_y:.1f}, {start_yaw:.2f})')
        images, labels = collect_episode(
            simulation,
            camera,
            controller,
            geometry,
            start_x,
            start_y,
            start_yaw,
        )
        X.extend(images)
        Y.extend(labels)
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    print(f'样本数: {len(X)}，图像 {X.shape[1:]}')

    img_mean = float(np.mean(X))
    img_std = float(np.std(X)) or 1.0
    X_normalized = (X - img_mean) / img_std

    print('训练小 CNN（纯 NumPy）...')
    net, history, _ = train_network(
        X_normalized,
        Y,
        architecture='small_cnn',
        epochs=25,
        batch_size=32,
        lr=1e-3,
        val_split=0.2,
        seed=42,
        verbose=True,
    )
    predictions = net.forward(X_normalized[-min(200, len(X)):])
    correlation_v = float(np.corrcoef(
        predictions[:, 0], Y[-min(200, len(Y)):, 0]
    )[0, 1])
    correlation_wz = float(np.corrcoef(
        predictions[:, 1], Y[-min(200, len(Y)):, 1]
    )[0, 1])
    print(f'最近 200 样本相关系数: v={correlation_v:.4f}, wz={correlation_wz:.4f}')

    output_path = PACKAGE_ROOT / 'models' / 'e2e_small_cnn_bc.npz'
    payload = {key: value.astype(np.float32)
               for key, value in net.state_dict().items()}
    payload['img_mean'] = np.array([img_mean], dtype=np.float32)
    payload['img_std'] = np.array([img_std], dtype=np.float32)
    np.savez(output_path, **payload)
    print(f'端到端权值已保存: {output_path}')

    with open(PACKAGE_ROOT / 'models' / 'e2e_training.csv', 'w') as f:
        f.write('epoch,train_loss,val_loss\n')
        for index, (train_loss, val_loss) in enumerate(zip(
            history['train'], history['val']
        ), start=1):
            f.write(f'{index},{train_loss:.6f},{val_loss:.6f}\n')


if __name__ == '__main__':
    main()
