#!/usr/bin/env python3
"""任务②神经网络测试：感知网络精度、控制网络精度、零权值消融。"""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import numpy as np  # noqa: E402

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.kinematics import DriveGeometry  # noqa: E402
from mujoco_auto_parking.parking_perception import (  # noqa: E402
    TARGET_SLOT_XY,
    ImagePerceptionNetwork,
    decode_output,
    encode_label,
    relative_slot_pose,
    wrap_angle,
)
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    TrajectoryController,
    evaluate_tracking,
)

MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'
PERCEPTION_WEIGHTS = PACKAGE_ROOT / 'models' / 'perception.npz'
TRAJECTORY_WEIGHTS = PACKAGE_ROOT / 'models' / 'trajectory.npz'


def perception_errors(network, samples=80, seed=7):
    rng = np.random.default_rng(seed)
    simulation = MujocoSimulation(MODEL_PATH)
    camera = CameraSensor(
        simulation.model, simulation.data, width=160, height=120
    )
    target_x, target_y = TARGET_SLOT_XY
    errors = []
    for _ in range(samples):
        x = float(rng.uniform(-5.0, 4.5))
        y = float(rng.uniform(-1.4, 1.4))
        bearing = math.atan2(target_y - y, target_x - x)
        yaw = bearing + float(rng.uniform(-0.6, 0.6))
        simulation.set_pose(x, y, yaw)
        rgb = camera.render_rgb()
        dx, dy, dtheta = network.predict(rgb)
        true_dx, true_dy, true_dtheta = relative_slot_pose(x, y, yaw)
        errors.append([
            abs(dx - true_dx),
            abs(dy - true_dy),
            abs(wrap_angle(dtheta - true_dtheta)),
        ])
    camera.close()
    return np.mean(np.asarray(errors), axis=0)


def check_perception() -> None:
    assert PERCEPTION_WEIGHTS.is_file(), PERCEPTION_WEIGHTS
    trained = ImagePerceptionNetwork(PERCEPTION_WEIGHTS)
    zero = ImagePerceptionNetwork(zero_weights=True)
    trained_error = perception_errors(trained)
    zero_error = perception_errors(zero)
    assert trained_error[0] < 0.5, trained_error
    assert trained_error[1] < 0.5, trained_error
    assert trained_error[2] < 0.5, trained_error
    assert trained_error[2] < zero_error[2], (trained_error, zero_error)
    print(
        '[PASS] 视觉感知网络：'
        f'MAE dx {trained_error[0]:.3f} m, dy {trained_error[1]:.3f} m, '
        f'dtheta {trained_error[2]:.3f} rad'
    )
    print(
        '[PASS] 感知零权值消融：'
        f'dtheta 误差 {zero_error[2]:.3f} rad（训练网络 {trained_error[2]:.3f}）'
    )


def check_trajectory_network() -> None:
    assert TRAJECTORY_WEIGHTS.is_file(), TRAJECTORY_WEIGHTS
    controller = TrajectoryController(TRAJECTORY_WEIGHTS)
    rng = np.random.default_rng(11)
    predictions = []
    targets = []
    for _ in range(1000):
        ex = rng.uniform(-0.6, 0.6)
        ey = rng.uniform(-0.5, 0.5)
        epsi = rng.uniform(-1.0, 1.0)
        velocity = rng.uniform(0.0, 0.8)
        v_ref = rng.uniform(0.3, 0.6)
        features = np.array([
            ex,
            ey / 0.5,
            epsi / math.pi,
            velocity,
            v_ref / 0.6,
        ])
        speed, yaw_rate = TrajectoryController.expert_from_features(
            features, v_ref
        )
        predictions.append(controller.model.forward(
            features.reshape(1, -1)
        )[0])
        targets.append([
            2.0 * speed / v_ref - 1.0,
            yaw_rate / 1.2,
        ])
    predictions = np.asarray(predictions)
    targets = np.asarray(targets)
    corr_v = float(np.corrcoef(predictions[:, 0], targets[:, 0])[0, 1])
    corr_wz = float(np.corrcoef(predictions[:, 1], targets[:, 1])[0, 1])
    assert corr_v > 0.95, corr_v
    assert corr_wz > 0.95, corr_wz
    print(
        f'[PASS] 轨迹控制网络：与专家相关系数 v={corr_v:.4f}, '
        f'wz={corr_wz:.4f}'
    )


def check_closed_loop() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    geometry = DriveGeometry(0.08, 0.48, 8.0)
    trained = TrajectoryController(TRAJECTORY_WEIGHTS)
    zero = TrajectoryController(zero_weights=True)
    trained_metrics = evaluate_tracking(trained, simulation, geometry)
    zero_metrics = evaluate_tracking(zero, simulation, geometry)
    assert trained_metrics['reached'] == 1.0, trained_metrics
    assert trained_metrics['rmse'] < 0.6, trained_metrics
    assert zero_metrics['rmse'] > trained_metrics['rmse'], (
        trained_metrics, zero_metrics
    )
    print(
        '[PASS] 闭环跟踪：'
        f'NN RMSE {trained_metrics["rmse"]:.3f} m '
        f'(到达 {bool(trained_metrics["reached"])})，'
        f'零权值 RMSE {zero_metrics["rmse"]:.3f} m '
        f'(到达 {bool(zero_metrics["reached"])})'
    )


def main() -> None:
    check_perception()
    check_trajectory_network()
    check_closed_loop()
    print('\n任务②神经网络测试全部通过。')


if __name__ == '__main__':
    main()
