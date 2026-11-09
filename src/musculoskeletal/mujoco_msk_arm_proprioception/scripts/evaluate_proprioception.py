#!/usr/bin/env python3
"""Evaluate proprioceptive noise, delay, and co-contraction stiffness."""

from __future__ import annotations

import argparse
from math import sin
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_msk_arm_proprioception.model import (  # noqa: E402
    MUSCLE_NAMES,
    ProprioceptionModel,
    coactivation_joint_stiffness,
)
from mujoco_msk_arm_proprioception.sensor import (  # noqa: E402
    NoisyDelayedProprioception,
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
FILL_PURPLE = '#f1ecf9'


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
    figure, axis = plt.subplots(figsize=(14, 7))
    figure.patch.set_facecolor('white')
    axis.set_xlim(0.0, 14.0)
    axis.set_ylim(0.0, 7.0)
    axis.axis('off')
    axis.set_title(
        'ROS 2 proprioception data flow',
        fontsize=16,
        weight='bold',
        color=INK,
        pad=14,
    )

    box(
        axis,
        1.9,
        3.8,
        2.75,
        2.35,
        '/mujoco_musculoskeletal',
        '_arm_reach',
        FILL_BLUE,
        BLUE,
    )
    input_topics = (
        (5.65, 5.25, '/joint_states', 'position + velocity'),
        (5.65, 3.8, '/muscle_forces', '6 forces'),
        (5.65, 2.35, '/muscle_activations', '6 activations'),
    )
    for x, y, title, detail in input_topics:
        box(axis, x, y, 2.55, 0.95, title, detail, FILL_ORANGE, ORANGE)
        arrow(axis, (3.30, 3.8), (4.35, y), ORANGE)

    box(
        axis,
        9.0,
        3.8,
        3.25,
        3.25,
        '/mujoco_msk_arm',
        '_proprioception',
        FILL_TEAL,
        TEAL,
    )
    for x, y, _title, _detail in input_topics:
        arrow(axis, (6.95, y), (7.35, 3.8), TEAL)

    axis.text(
        9.0,
        2.76,
        'series tendon + spindle + GTO + noise + delay',
        ha='center',
        fontsize=8.8,
        color=INK,
    )
    outputs = (
        (12.25, 5.25, 'spindle_length', '6 lengths'),
        (12.25, 3.95, 'spindle_velocity', '6 velocities'),
        (12.25, 2.65, 'golgi_force', '6 forces'),
        (12.25, 1.35, 'tendon_extension', '6 extensions'),
    )
    for x, y, title, detail in outputs:
        box(axis, x, y, 2.9, 0.92, title, detail, FILL_PURPLE, PURPLE)
        arrow(axis, (10.65, 3.8), (10.8, y), PURPLE)

    axis.text(
        7.0,
        0.45,
        'All sensory outputs are std_msgs/Float64MultiArray in muscle order.',
        ha='center',
        fontsize=9.2,
        color=MUTED,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)


def simulate_force_trace(delay_s: float, seed: int = 12):
    model = ProprioceptionModel()
    sensor = NoisyDelayedProprioception(
        sample_period_s=0.01,
        delay_s=delay_s,
        length_noise_std_m=0.0002,
        velocity_noise_std_m_s=0.002,
        force_noise_std_n=0.15,
        tendon_extension_noise_std_m=0.00005,
        seed=seed,
    )
    times = []
    true_forces = []
    sensed_forces = []
    for index in range(220):
        time_s = index * 0.01
        shoulder = 0.12 * sin(2.0 * np.pi * 0.5 * time_s)
        elbow = 1.25 + 0.18 * sin(
            2.0 * np.pi * 0.5 * time_s + 0.7
        )
        shoulder_velocity = (
            0.12
            * 2.0
            * np.pi
            * 0.5
            * np.cos(2.0 * np.pi * 0.5 * time_s)
        )
        elbow_velocity = (
            0.18
            * 2.0
            * np.pi
            * 0.5
            * np.cos(2.0 * np.pi * 0.5 * time_s + 0.7)
        )
        force = 24.0 + 7.0 * sin(
            2.0 * np.pi * 1.1 * time_s
        ) + np.array([0.0, 0.8, 1.5, 2.0, 2.5, 3.0])
        activation = np.clip(force / 80.0, 0.0, 1.0)
        sample = model.estimate(
            shoulder_rad=shoulder,
            elbow_rad=elbow,
            shoulder_velocity_rad_s=shoulder_velocity,
            elbow_velocity_rad_s=elbow_velocity,
            activations=activation,
            muscle_forces_n=force,
            dt_s=0.01,
        )
        sensed = sensor.measure(sample)
        times.append(time_s)
        true_forces.append(float(sample.golgi_force_n[0]))
        sensed_forces.append(float(sensed.golgi_force_n[0]))
    return np.asarray(times), np.asarray(true_forces), np.asarray(sensed_forces)


def delay_rmse(delay_s: float, seed: int = 13) -> float:
    _times, true_force, sensed_force = simulate_force_trace(delay_s, seed)
    delay_samples = int(round(delay_s / 0.01))
    start = delay_samples + 1
    reference = true_force[start:]
    measured = sensed_force[start:]
    return float(np.sqrt(np.mean((measured - reference) ** 2)))


def generate_benchmark(output: Path) -> dict[str, float]:
    coactivation_levels = np.linspace(0.0, 0.40, 41)
    stiffness = np.array([
        np.linalg.eigvalsh(
            coactivation_joint_stiffness(float(level))
        )[0]
        for level in coactivation_levels
    ])
    delays = np.array([0.0, 0.01, 0.02, 0.03, 0.05, 0.08])
    rmse = np.array([delay_rmse(float(delay)) for delay in delays])
    times, true_force, sensed_force = simulate_force_trace(
        delay_s=0.04,
        seed=21,
    )

    figure, axes = plt.subplots(
        1,
        3,
        figsize=(15, 4.8),
        gridspec_kw={'width_ratios': (1.0, 1.0, 1.35)},
    )
    figure.patch.set_facecolor('white')
    figure.suptitle(
        'Proprioception benchmarks',
        fontsize=15,
        weight='bold',
        color=INK,
    )

    axes[0].plot(
        coactivation_levels,
        stiffness,
        color=BLUE,
        linewidth=2.4,
    )
    axes[0].fill_between(
        coactivation_levels,
        stiffness,
        color=FILL_BLUE,
        alpha=0.7,
    )
    axes[0].set_xlabel('coactivation', color=INK)
    axes[0].set_ylabel('minimum joint stiffness', color=INK)
    axes[0].set_title('Co-contraction stiffness', loc='left', color=INK)

    axes[1].plot(
        1000.0 * delays,
        rmse,
        marker='o',
        color=ORANGE,
        linewidth=2.2,
    )
    axes[1].set_xlabel('transport delay / ms', color=INK)
    axes[1].set_ylabel('force RMSE / N', color=INK)
    axes[1].set_title('Delay degrades fidelity', loc='left', color=INK)

    axes[2].plot(
        times,
        true_force,
        color=TEAL,
        linewidth=2.0,
        label='true force',
    )
    axes[2].plot(
        times,
        sensed_force,
        color=PURPLE,
        linewidth=1.8,
        label='sensed force (40 ms delay)',
    )
    axes[2].set_xlabel('time / s', color=INK)
    axes[2].set_ylabel('Golgi force / N', color=INK)
    axes[2].set_title('Noisy delayed signal', loc='left', color=INK)
    axes[2].legend(frameon=False, fontsize=8.5, loc='best')

    for axis in axes:
        axis.grid(True, color=GRID, linewidth=0.7)
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
        axis.tick_params(colors=MUTED)
    figure.tight_layout(rect=(0.01, 0.01, 0.99, 0.92))
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    metrics = {
        'stiffness_at_0_02': float(
            np.linalg.eigvalsh(coactivation_joint_stiffness(0.02))[0]
        ),
        'stiffness_at_0_40': float(
            np.linalg.eigvalsh(coactivation_joint_stiffness(0.40))[0]
        ),
        'rmse_at_0_ms_n': float(rmse[0]),
        'rmse_at_40_ms_n': float(delay_rmse(0.04)),
        'rmse_at_80_ms_n': float(rmse[-1]),
    }
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=(
            REPO_ROOT
            / 'docs'
            / 'img'
            / 'musculoskeletal'
            / 'msk_arm_proprioception'
        ),
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    architecture = args.output_dir / 'architecture.png'
    benchmark = args.output_dir / 'proprioception_benchmark.png'
    generate_architecture(architecture)
    metrics = generate_benchmark(benchmark)
    print('architecture=%s' % architecture)
    print('benchmark=%s' % benchmark)
    for name, value in metrics.items():
        print('%s=%.6f' % (name, value))


if __name__ == '__main__':
    main()
