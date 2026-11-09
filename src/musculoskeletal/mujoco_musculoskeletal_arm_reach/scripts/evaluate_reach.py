#!/usr/bin/env python3
"""Evaluate the closed-loop arm reaching controller."""

from __future__ import annotations

import argparse
from math import hypot
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_musculoskeletal_arm_reach.controller import (  # noqa: E402
    ArmControlState,
    MuscleArmController,
)
from mujoco_musculoskeletal_arm_reach.simulation import ArmSimulation  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--target-x', type=float, default=0.38)
    parser.add_argument('--target-z', type=float, default=-0.17)
    parser.add_argument('--steps', type=int, default=1600)
    parser.add_argument(
        '--output',
        type=Path,
        default=(
            REPO_ROOT
            / 'docs'
            / 'img'
            / 'musculoskeletal'
            / 'msk_arm_reach'
            / 'reach_trajectory.png'
        ),
    )
    args = parser.parse_args()

    simulation = ArmSimulation(PACKAGE_ROOT / 'models' / 'arm.xml')
    controller = MuscleArmController()
    simulation.reset(-0.25, 0.75)
    target = (args.target_x, args.target_z)
    trajectory = []
    for _ in range(args.steps):
        observation = simulation.observe()
        trajectory.append(
            (observation.end_effector_x_m, observation.end_effector_z_m)
        )
        command = controller.command(
            ArmControlState(
                target_x_m=target[0],
                target_z_m=target[1],
                shoulder_rad=observation.shoulder_rad,
                elbow_rad=observation.elbow_rad,
                shoulder_velocity_rad_s=(
                    observation.shoulder_velocity_rad_s
                ),
                elbow_velocity_rad_s=observation.elbow_velocity_rad_s,
            )
        )
        observation = simulation.step(command)
    trajectory.append(
        (observation.end_effector_x_m, observation.end_effector_z_m)
    )
    error = hypot(
        trajectory[-1][0] - target[0],
        trajectory[-1][1] - target[1],
    )

    figure, axis = plt.subplots(figsize=(7.5, 5.0))
    figure.patch.set_facecolor('white')
    x_values = [point[0] for point in trajectory]
    z_values = [point[1] for point in trajectory]
    axis.plot(x_values, z_values, color='#2864dc', linewidth=2.5)
    axis.scatter(x_values[0], z_values[0], color='#65758a', s=55, label='start')
    axis.scatter(x_values[-1], z_values[-1], color='#d3562d', s=70, label='final')
    axis.scatter(target[0], target[1], color='#169664', marker='x', s=120,
                 linewidth=3, label='target')
    axis.set_xlabel('x / m')
    axis.set_ylabel('z / m')
    axis.set_title('Muscle-driven closed-loop reach', weight='bold')
    axis.axis('equal')
    axis.grid(True, color='#dce4ee', linewidth=0.7)
    axis.legend(frameon=False, loc='best')
    axis.text(
        0.02,
        0.02,
        'final error = %.4f m' % error,
        transform=axis.transAxes,
        fontsize=10,
        color='#27364a',
    )
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    print('saved=%s' % args.output)
    print('final_error_m=%.5f' % error)


if __name__ == '__main__':
    main()
