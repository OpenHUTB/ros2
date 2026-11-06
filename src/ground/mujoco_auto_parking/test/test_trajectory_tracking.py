#!/usr/bin/env python3
"""Evaluate the neural trajectory controller against the zero-weight ablation."""

import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_auto_parking.kinematics import DriveGeometry  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402
from mujoco_auto_parking.trajectory_control import (  # noqa: E402
    TrajectoryController,
    evaluate_tracking,
)

WEIGHTS = PACKAGE_ROOT / 'models' / 'trajectory.npz'


def format_metrics(name: str, metrics) -> str:
    return (
        f'{name:12s} RMSE={metrics["rmse"]:.3f} m  '
        f'max={metrics["max_error"]:.3f} m  '
        f'heading={metrics["rmse_heading"]:.3f} rad  '
        f'time={metrics["time"]:.2f} s  '
        f'reached={bool(metrics["reached"])}'
    )


def main() -> None:
    assert WEIGHTS.is_file(), f'缺少轨迹控制权值: {WEIGHTS}'
    simulation = MujocoSimulation(PACKAGE_ROOT / 'models' / 'parking_lot.xml')
    geometry = DriveGeometry(0.08, 0.48, 8.0)
    trained = TrajectoryController(WEIGHTS)
    zero = TrajectoryController(zero_weights=True)

    trained_metrics = evaluate_tracking(trained, simulation, geometry)
    zero_metrics = evaluate_tracking(zero, simulation, geometry)
    print('给定 S 曲线轨迹跟踪（v_ref = 0.45 m/s）')
    print(format_metrics('NN', trained_metrics))
    print(format_metrics('零权值', zero_metrics))
    assert trained_metrics['reached'] == 1.0
    assert trained_metrics['rmse'] < 0.6
    assert zero_metrics['rmse'] > trained_metrics['rmse']
    print('\n结论：训练网络能到达终点，零权值网络跑偏；控制量确实来自网络。')


if __name__ == '__main__':
    main()
