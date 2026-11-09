#!/usr/bin/env python3
"""Compare fixed control with online fatigue-learning compensation."""

from __future__ import annotations

import argparse
from math import hypot
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
ARM_PACKAGE = (
    REPO_ROOT
    / 'src'
    / 'musculoskeletal'
    / 'mujoco_musculoskeletal_arm_reach'
)
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(ARM_PACKAGE))

from mujoco_musculoskeletal_arm_reach.controller import (  # noqa: E402
    ArmControlState,
    MuscleArmController,
)
from mujoco_musculoskeletal_arm_reach.kinematics import (  # noqa: E402
    inverse_kinematics,
)
from mujoco_musculoskeletal_arm_reach.simulation import ArmSimulation  # noqa: E402
from mujoco_msk_arm_fatigue_learning.fatigue import (  # noqa: E402
    MuscleFatigueModel,
)
from mujoco_msk_arm_fatigue_learning.learning import (  # noqa: E402
    AdaptiveFatigueCompensator,
)


INK = '#23354d'
MUTED = '#5d6d80'
GRID = '#dbe3ec'
BLUE = '#2f65b0'
TEAL = '#138a7b'
ORANGE = '#d06a2f'
PURPLE = '#7451a6'
FILL_BLUE = '#eaf2fd'
FILL_TEAL = '#e8f6f2'
FILL_ORANGE = '#fdf0e7'


def box(axis, x, y, width, height, title, detail, face, edge):
    axis.add_patch(
        FancyBboxPatch(
            (x - width / 2.0, y - height / 2.0),
            width,
            height,
            boxstyle='round,pad=0.025,rounding_size=0.12',
            linewidth=1.5,
            edgecolor=edge,
            facecolor=face,
        )
    )
    axis.text(
        x,
        y + 0.10,
        title,
        ha='center',
        va='center',
        fontsize=10.0,
        weight='bold',
        color=INK,
    )
    axis.text(
        x,
        y - 0.16,
        detail,
        ha='center',
        va='center',
        fontsize=8.2,
        color=MUTED,
    )


def arrow(axis, start, end, color):
    axis.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle='-|>',
            mutation_scale=13,
            linewidth=1.6,
            color=color,
            shrinkA=4,
            shrinkB=4,
        )
    )


def generate_architecture(output: Path) -> None:
    figure, axis = plt.subplots(figsize=(15, 7.5))
    figure.patch.set_facecolor('white')
    axis.set_xlim(0.0, 15.0)
    axis.set_ylim(0.0, 7.5)
    axis.axis('off')
    axis.set_title(
        'Fatigue learning control data flow',
        fontsize=16,
        weight='bold',
        color=INK,
        pad=14,
    )
    box(
        axis,
        1.8,
        4.0,
        2.7,
        1.4,
        '/target_position',
        'geometry_msgs/Point',
        FILL_ORANGE,
        ORANGE,
    )
    box(
        axis,
        5.7,
        4.0,
        3.7,
        3.7,
        '/mujoco_msk_arm',
        '_fatigue_learning',
        FILL_TEAL,
        TEAL,
    )
    axis.text(
        5.7,
        3.0,
        'IK + PD + fatigue + online gain learning + MuJoCo',
        ha='center',
        fontsize=8.8,
        color=INK,
    )
    box(
        axis,
        10.3,
        6.0,
        3.1,
        1.15,
        'State topics',
        'joints, tip, force, activation',
        FILL_BLUE,
        BLUE,
    )
    box(
        axis,
        10.3,
        4.0,
        3.1,
        1.15,
        'fatigue/state',
        'six muscle fatigue values',
        FILL_BLUE,
        BLUE,
    )
    box(
        axis,
        10.3,
        2.0,
        3.1,
        1.15,
        'compensation_gain',
        'six learned gains',
        FILL_BLUE,
        BLUE,
    )
    box(
        axis,
        6.2,
        6.9,
        2.8,
        1.0,
        'policy.json',
        'offline tuned',
        FILL_ORANGE,
        ORANGE,
    )
    arrow(axis, (3.15, 4.0), (3.85, 4.0), ORANGE)
    for y in (6.0, 4.0, 2.0):
        arrow(axis, (7.55, 4.0), (8.75, y), BLUE)
    arrow(axis, (6.2, 6.38), (6.2, 5.87), ORANGE)
    axis.text(
        7.5,
        0.7,
        'The node owns the MuJoCo simulation; policy loading overrides YAML gains.',
        ha='center',
        fontsize=9.3,
        color=MUTED,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)


def run_episode(
    enable_learning: bool,
    duration_s: float = 30.0,
    learning_rate: float = 100.0,
    gain_decay_rate: float = 0.20,
    maximum_gain: float = 2.0,
    payload_kg: float = 1.0,
):
    simulation = ArmSimulation(
        ARM_PACKAGE / 'models' / 'arm.xml',
        payload_kg=payload_kg,
    )
    controller = MuscleArmController()
    fatigue_model = MuscleFatigueModel()
    compensator = AdaptiveFatigueCompensator(
        learning_rate=learning_rate,
        gain_decay_rate=gain_decay_rate,
        error_deadband_rad=0.001,
        maximum_gain=maximum_gain,
    )
    simulation.reset(-0.25, 0.75)
    target = (0.38, -0.17)
    target_joint = np.asarray(
        inverse_kinematics(*target, controller.geometry)
    )
    times = []
    errors = []
    fatigue_trace = []
    gain_trace = []
    ticks = int(duration_s / 0.01)
    for tick in range(ticks):
        observation = simulation.observe()
        base_activation = np.asarray(
            controller.command(
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
        )
        fatigue_state = fatigue_model.update(base_activation, 0.01)
        if enable_learning:
            joint_error = target_joint - np.array(
                [observation.shoulder_rad, observation.elbow_rad]
            )
            gain = compensator.update(
                base_activation,
                joint_error,
                0.01,
            )
            effective_activation = compensator.compensate(
                base_activation,
                fatigue_state.strength_retention,
            )
        else:
            gain = np.ones(6)
            effective_activation = (
                base_activation * fatigue_state.strength_retention
            )
        for _ in range(5):
            observation = simulation.step(tuple(effective_activation))
        times.append((tick + 1) * 0.01)
        errors.append(
            hypot(
                observation.end_effector_x_m - target[0],
                observation.end_effector_z_m - target[1],
            )
        )
        fatigue_trace.append(float(np.mean(fatigue_state.fatigue)))
        gain_trace.append(float(np.mean(gain)))
    return {
        'time': np.asarray(times),
        'error': np.asarray(errors),
        'fatigue': np.asarray(fatigue_trace),
        'gain': np.asarray(gain_trace),
    }


def generate_figure(output: Path) -> dict[str, float]:
    fixed = run_episode(enable_learning=False)
    learned = run_episode(enable_learning=True)
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(11, 9),
        sharex=True,
    )
    figure.patch.set_facecolor('white')
    figure.suptitle(
        'Fatigue and online learning compensation',
        fontsize=16,
        weight='bold',
        color=INK,
    )
    axes[0].plot(
        fixed['time'],
        fixed['error'],
        color=ORANGE,
        linewidth=2.0,
        label='fixed activation',
    )
    axes[0].plot(
        learned['time'],
        learned['error'],
        color=BLUE,
        linewidth=2.0,
        label='learned compensation',
    )
    axes[0].set_ylabel('tip error / m', color=INK)
    axes[0].set_title('Endpoint tracking error', loc='left', color=INK)
    axes[0].legend(frameon=False, loc='upper left')

    axes[1].plot(
        fixed['time'],
        fixed['fatigue'],
        color=PURPLE,
        linewidth=2.0,
    )
    axes[1].set_ylabel('mean fatigue', color=INK)
    axes[1].set_title('Fatigue accumulation', loc='left', color=INK)

    axes[2].plot(
        learned['time'],
        learned['gain'],
        color=TEAL,
        linewidth=2.0,
    )
    axes[2].set_ylabel('mean gain', color=INK)
    axes[2].set_xlabel('simulation time / s', color=INK)
    axes[2].set_title('Learned compensation gain', loc='left', color=INK)

    for axis in axes:
        axis.grid(True, color=GRID, linewidth=0.7)
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
        axis.tick_params(colors=MUTED)
    figure.tight_layout(rect=(0.02, 0.02, 0.98, 0.94))
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    tail = slice(-200, None)
    return {
        'fixed_tail_error_m': float(np.mean(fixed['error'][tail])),
        'learned_tail_error_m': float(np.mean(learned['error'][tail])),
        'final_mean_fatigue': float(fixed['fatigue'][-1]),
        'final_mean_gain': float(learned['gain'][-1]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        type=Path,
        default=(
            REPO_ROOT
            / 'docs'
            / 'img'
            / 'musculoskeletal'
            / 'msk_arm_fatigue_learning'
            / 'fatigue_learning.png'
        ),
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    architecture = args.output.parent / 'architecture.png'
    generate_architecture(architecture)
    metrics = generate_figure(args.output)
    print('architecture=%s' % architecture)
    print('output=%s' % args.output)
    for name, value in metrics.items():
        print('%s=%.6f' % (name, value))


if __name__ == '__main__':
    main()
