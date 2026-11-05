#!/usr/bin/env python3
"""Run the reaching controller and render a MuJoCo documentation preview."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image


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
            / 'arm_mujoco_preview.png'
        ),
    )
    args = parser.parse_args()

    simulation = ArmSimulation(PACKAGE_ROOT / 'models' / 'arm.xml')
    controller = MuscleArmController()
    simulation.reset(-0.25, 0.75)
    target = (0.38, -0.17)
    for _ in range(args.steps):
        observation = simulation.observe()
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

    renderer = mujoco.Renderer(
        simulation.model,
        height=720,
        width=1280,
    )
    renderer.update_scene(simulation.data, camera='front')
    pixels = renderer.render()
    renderer.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.asarray(pixels, dtype=np.uint8)).save(
        args.output,
        optimize=True,
    )
    print('saved=%s' % args.output)
    print(
        'tip=(%.4f, %.4f) target=(%.2f, %.2f) time=%.3f'
        % (
            observation.end_effector_x_m,
            observation.end_effector_z_m,
            target[0],
            target[1],
            observation.time_s,
        )
    )


if __name__ == '__main__':
    main()
