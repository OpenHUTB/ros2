#!/usr/bin/env python3
"""任务④测试：图像端到端控制、闭环跟踪和零权值消融。"""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import numpy as np  # noqa: E402

from mujoco_auto_parking.camera_sensor import CameraSensor  # noqa: E402
from mujoco_auto_parking.end_to_end_networks import (  # noqa: E402
    VisuomotorController,
)
from mujoco_auto_parking.kinematics import DriveGeometry  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    ReferenceTrajectory,
    build_s_curve,
)

MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'
WEIGHTS = PACKAGE_ROOT / 'models' / 'e2e_small_cnn_bc.npz'


def closed_loop(controller, simulation, camera, geometry, max_time=20.0):
    start_x, start_y, start_yaw = -6.0, 0.0, 0.0
    trajectory = ReferenceTrajectory(
        build_s_curve(start_x, start_y, start_yaw)
    )
    simulation.reset()
    simulation.set_pose(start_x, start_y, start_yaw)
    errors = []
    latencies = []
    control_period = 0.02
    steps = max(1, int(round(control_period / simulation.timestep)))
    time_elapsed = 0.0
    reached = False
    while time_elapsed < max_time:
        state = simulation.planar_state()
        index, distance, _ = trajectory.lookahead(state.x, state.y)
        rgb = camera.render_rgb()
        result = controller.predict_action(rgb)
        latencies.append(result['latency_ms'])
        left, right = geometry.wheel_speeds(result['v'], result['wz'])
        for _ in range(steps):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()
        errors.append(distance)
        time_elapsed += control_period
        if index >= len(trajectory.points) - 4 and distance < 0.25:
            reached = True
            break
    errors = np.asarray(errors)
    return {
        'rmse': float(np.sqrt(np.mean(errors ** 2))),
        'max_error': float(np.max(errors)),
        'time': float(time_elapsed),
        'reached': float(1.0 if reached else 0.0),
        'latency_ms': float(np.mean(latencies)),
    }


def main() -> None:
    assert WEIGHTS.is_file(), f'缺少端到端权值: {WEIGHTS}'
    simulation = MujocoSimulation(MODEL_PATH)
    camera = CameraSensor(
        simulation.model, simulation.data, width=160, height=120
    )
    geometry = DriveGeometry(0.08, 0.48, 8.0)
    trained = VisuomotorController(
        architecture='small_cnn', weights_path=WEIGHTS
    )
    zero = VisuomotorController(
        architecture='small_cnn', zero_weights=True
    )

    trained_metrics = closed_loop(trained, simulation, camera, geometry)
    zero_metrics = closed_loop(zero, simulation, camera, geometry)
    print('端到端视觉控制（S 曲线，闭环）')
    print(
        f'CNN       RMSE={trained_metrics["rmse"]:.3f} m  '
        f'max={trained_metrics["max_error"]:.3f} m  '
        f'latency={trained_metrics["latency_ms"]:.1f} ms  '
        f'reached={bool(trained_metrics["reached"])}'
    )
    print(
        f'零权值    RMSE={zero_metrics["rmse"]:.3f} m  '
        f'max={zero_metrics["max_error"]:.3f} m  '
        f'latency={zero_metrics["latency_ms"]:.1f} ms  '
        f'reached={bool(zero_metrics["reached"])}'
    )
    assert trained_metrics['reached'] == 1.0, trained_metrics
    assert trained_metrics['rmse'] < 0.7, trained_metrics
    assert trained_metrics['latency_ms'] < 100.0, trained_metrics
    assert zero_metrics['rmse'] > trained_metrics['rmse'], (
        trained_metrics, zero_metrics
    )
    print('\n任务④端到端测试通过：控制量来自图像，零权值明显变差。')


if __name__ == '__main__':
    main()
