#!/usr/bin/env python3
"""Generate ROS graph, runtime logic, and closed-loop signal figures."""

from __future__ import annotations

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
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_musculoskeletal_arm_reach.controller import (  # noqa: E402
    ArmControlState,
    MuscleArmController,
)
from mujoco_musculoskeletal_arm_reach.kinematics import (  # noqa: E402
    inverse_kinematics,
)
from mujoco_musculoskeletal_arm_reach.simulation import ArmSimulation  # noqa: E402


OUTPUT_DIR = (
    REPO_ROOT
    / 'docs'
    / 'img'
    / 'musculoskeletal'
    / 'msk_arm_reach'
)

INK = '#23354d'
MUTED = '#5d6d80'
GRID = '#dbe3ec'
BLUE = '#2f65b0'
TEAL = '#138a7b'
ORANGE = '#d06a2f'
RED = '#b94743'
PURPLE = '#7451a6'
FILL_BLUE = '#eaf2fd'
FILL_TEAL = '#e8f6f2'
FILL_ORANGE = '#fdf0e7'
FILL_PURPLE = '#f1ecf9'


def box(
    axis,
    x,
    y,
    width,
    height,
    title,
    detail='',
    face=FILL_BLUE,
    edge=BLUE,
):
    patch = FancyBboxPatch(
        (x - width / 2.0, y - height / 2.0),
        width,
        height,
        boxstyle='round,pad=0.025,rounding_size=0.12',
        linewidth=1.5,
        edgecolor=edge,
        facecolor=face,
    )
    axis.add_patch(patch)
    axis.text(
        x,
        y + (0.11 if detail else 0.0),
        title,
        ha='center',
        va='center',
        fontsize=10.5,
        weight='bold',
        color=INK,
    )
    if detail:
        axis.text(
            x,
            y - 0.15,
            detail,
            ha='center',
            va='center',
            fontsize=8.4,
            color=MUTED,
        )


def arrow(axis, start, end, color=BLUE, rad=0.0):
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle='-|>',
        mutation_scale=13,
        linewidth=1.6,
        color=color,
        connectionstyle='arc3,rad=%s' % rad,
        shrinkA=4,
        shrinkB=4,
    )
    axis.add_patch(patch)


def generate_ros_graph() -> Path:
    output = OUTPUT_DIR / 'ros_graph.png'
    figure, axis = plt.subplots(figsize=(15, 8.0))
    figure.patch.set_facecolor('white')
    axis.set_xlim(0.0, 15.0)
    axis.set_ylim(0.0, 8.0)
    axis.axis('off')
    axis.set_title(
        'ROS 2 graph: one node, one input topic, four output topics',
        fontsize=16,
        weight='bold',
        color=INK,
        pad=16,
    )

    box(
        axis,
        1.55,
        4.9,
        2.45,
        1.35,
        'Target source',
        'geometry_msgs/msg/Point',
        FILL_ORANGE,
        ORANGE,
    )
    box(
        axis,
        4.55,
        4.9,
        2.55,
        1.35,
        '/target_position',
        'subscriber',
        FILL_BLUE,
        BLUE,
    )
    box(
        axis,
        7.85,
        4.9,
        3.15,
        2.0,
        '/mujoco_musculoskeletal_arm_reach',
        'IK -> PD -> synergy -> MuJoCo',
        FILL_TEAL,
        TEAL,
    )
    arrow(axis, (2.80, 4.9), (3.25, 4.9), ORANGE)
    arrow(axis, (5.85, 4.9), (6.25, 4.9), BLUE)

    output_specs = (
        (7.0, '/joint_states', 'sensor_msgs/msg/JointState', BLUE, FILL_BLUE),
        (
            5.45,
            '/end_effector_position',
            'geometry_msgs/msg/Point',
            TEAL,
            FILL_TEAL,
        ),
        (
            3.9,
            '/muscle_activations',
            'std_msgs/msg/Float64MultiArray',
            ORANGE,
            FILL_ORANGE,
        ),
        (
            2.35,
            '/muscle_forces',
            'std_msgs/msg/Float64MultiArray',
            PURPLE,
            FILL_PURPLE,
        ),
    )
    for y, title, detail, color, face in output_specs:
        box(axis, 12.25, y, 3.15, 1.15, title, detail, face, color)
        arrow(axis, (9.45, 4.9), (10.65, y), color, rad=-0.07)
        axis.text(
            11.0,
            y + 0.42,
            'publisher',
            fontsize=8.2,
            color=MUTED,
        )

    axis.text(
        7.85,
        7.25,
        'Control rate: 100 Hz    Physics: 5 x 2 ms per timer tick',
        ha='center',
        fontsize=10.5,
        color=INK,
    )
    axis.text(
        7.85,
        0.8,
        'No services, actions, TF, or additional ROS topics are created by this node.',
        ha='center',
        fontsize=9.5,
        color=MUTED,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    return output


def generate_runtime_logic() -> Path:
    output = OUTPUT_DIR / 'runtime_logic.png'
    figure, axis = plt.subplots(figsize=(15, 8.0))
    figure.patch.set_facecolor('white')
    axis.set_xlim(0.0, 15.0)
    axis.set_ylim(0.0, 8.0)
    axis.axis('off')
    axis.set_title(
        'Runtime logic: launch, target handling, and timer-driven loop',
        fontsize=16,
        weight='bold',
        color=INK,
        pad=16,
    )

    top_nodes = (
        (1.55, 'ros2 launch', 'main.launch.py'),
        (4.55, 'Load YAML', 'simulation.yaml'),
        (7.55, 'Build model', 'models/arm.xml'),
        (10.55, 'Create ROS I/O', '1 sub + 4 pubs'),
        (13.55, '100 Hz timer', '10 ms period'),
    )
    for x, title, detail in top_nodes:
        box(axis, x, 6.65, 2.45, 1.15, title, detail, FILL_BLUE, BLUE)
    for left, right in zip(top_nodes[:-1], top_nodes[1:]):
        arrow(axis, (left[0] + 1.23, 6.65), (right[0] - 1.23, 6.65), BLUE)

    box(
        axis,
        1.9,
        4.75,
        2.85,
        1.35,
        '/target_position',
        'finite x/z + IK validation',
        FILL_ORANGE,
        ORANGE,
    )
    box(
        axis,
        5.15,
        4.75,
        2.75,
        1.35,
        'Reject or update',
        'invalid target is logged',
        FILL_ORANGE,
        ORANGE,
    )
    arrow(axis, (3.35, 4.75), (3.75, 4.75), ORANGE)
    arrow(axis, (5.15, 4.08), (4.35, 3.17), ORANGE, rad=-0.1)

    loop_nodes = (
        (1.45, 'Read state', 'q, qdot, forces'),
        (4.35, 'Cartesian IK', 'elbow-up solution'),
        (7.25, 'Joint PD', 'shoulder + elbow'),
        (10.15, 'Muscle synergy', '6 activations in [0, 1]'),
        (13.05, '5 physics steps', '10 ms simulated time'),
    )
    for x, title, detail in loop_nodes:
        box(axis, x, 2.55, 2.45, 1.2, title, detail, FILL_TEAL, TEAL)
    for left, right in zip(loop_nodes[:-1], loop_nodes[1:]):
        arrow(axis, (left[0] + 1.23, 2.55), (right[0] - 1.23, 2.55), TEAL)

    box(
        axis,
        8.15,
        0.75,
        5.45,
        0.9,
        'Publish joint_states, end_effector_position, muscle_activations, muscle_forces',
        '',
        FILL_PURPLE,
        PURPLE,
    )
    arrow(axis, (13.05, 1.94), (11.0, 1.21), PURPLE)
    arrow(axis, (5.42, 1.21), (1.45, 1.94), PURPLE, rad=0.18)
    axis.text(
        3.2,
        1.05,
        'next timer tick',
        fontsize=8.5,
        color=PURPLE,
        ha='center',
    )

    box(
        axis,
        13.65,
        4.75,
        1.9,
        1.35,
        'Exception',
        'safe stop',
        '#f8e8e7',
        RED,
    )
    arrow(axis, (13.05, 3.15), (13.65, 4.08), RED, rad=0.15)
    axis.text(
        7.7,
        3.55,
        'controller update runs once per timer tick',
        ha='center',
        fontsize=9.4,
        color=MUTED,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    return output


def generate_runtime_signals() -> Path:
    output = OUTPUT_DIR / 'runtime_signals.png'
    model_path = PACKAGE_ROOT / 'models' / 'arm.xml'
    simulation = ArmSimulation(model_path)
    controller = MuscleArmController()
    simulation.reset(-0.25, 0.75)
    target = (0.38, -0.17)
    target_shoulder, target_elbow = inverse_kinematics(
        *target,
        controller.geometry,
    )

    times = []
    errors = []
    shoulder = []
    elbow = []
    flexor = []
    extensor = []
    mean_force = []
    for tick in range(640):
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
        for _ in range(5):
            observation = simulation.step(command)
        times.append((tick + 1) * 0.01)
        errors.append(
            hypot(
                observation.end_effector_x_m - target[0],
                observation.end_effector_z_m - target[1],
            )
        )
        shoulder.append(observation.shoulder_rad)
        elbow.append(observation.elbow_rad)
        flexor.append(command[0] + command[2] + command[4])
        extensor.append(command[1] + command[3] + command[5])
        mean_force.append(np.mean(observation.muscle_forces_n))

    times_array = np.asarray(times)
    errors_array = np.asarray(errors)
    settled = np.flatnonzero(errors_array < 0.02)
    settle_time = float(times_array[settled[0]]) if settled.size else float('nan')

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 9),
        sharex=True,
        gridspec_kw={'height_ratios': (1.0, 1.2, 1.0)},
    )
    figure.patch.set_facecolor('white')
    figure.suptitle(
        'Closed-loop runtime behavior from the MuJoCo simulation',
        fontsize=16,
        weight='bold',
        color=INK,
    )

    axes[0].plot(times_array, errors_array, color=BLUE, linewidth=2.2)
    axes[0].axhline(0.02, color=GRID, linewidth=1.2)
    axes[0].axvline(settle_time, color=ORANGE, linewidth=1.3, linestyle='--')
    axes[0].text(
        settle_time + 0.04,
        0.021,
        'error < 0.02 m at %.2f s' % settle_time,
        color=ORANGE,
        fontsize=9,
    )
    axes[0].set_ylabel('tip error / m', color=INK)
    axes[0].set_ylim(bottom=0.0)
    axes[0].set_title(
        'Cartesian tracking error',
        loc='left',
        fontsize=11,
        color=INK,
    )

    axes[1].plot(
        times_array,
        shoulder,
        color=BLUE,
        linewidth=2.0,
        label='shoulder actual',
    )
    axes[1].plot(
        times_array,
        elbow,
        color=TEAL,
        linewidth=2.0,
        label='elbow actual',
    )
    axes[1].axhline(
        target_shoulder,
        color=BLUE,
        linewidth=1.2,
        linestyle='--',
        label='shoulder target',
    )
    axes[1].axhline(
        target_elbow,
        color=TEAL,
        linewidth=1.2,
        linestyle='--',
        label='elbow target',
    )
    axes[1].set_ylabel('joint angle / rad', color=INK)
    axes[1].set_title(
        'Joint trajectories',
        loc='left',
        fontsize=11,
        color=INK,
    )
    axes[1].legend(
        ncol=2,
        frameon=False,
        fontsize=8.5,
        loc='best',
    )

    axes[2].plot(
        times_array,
        flexor,
        color=ORANGE,
        linewidth=2.0,
        label='flexor activation sum',
    )
    axes[2].plot(
        times_array,
        extensor,
        color=PURPLE,
        linewidth=2.0,
        label='extensor activation sum',
    )
    force_axis = axes[2].twinx()
    force_axis.plot(
        times_array,
        mean_force,
        color=MUTED,
        linewidth=1.5,
        linestyle=':',
        label='mean muscle force',
    )
    force_axis.set_ylabel('mean force / N', color=MUTED)
    force_axis.tick_params(axis='y', colors=MUTED)
    lines, labels = axes[2].get_legend_handles_labels()
    force_lines, force_labels = force_axis.get_legend_handles_labels()
    axes[2].legend(
        lines + force_lines,
        labels + force_labels,
        ncol=3,
        frameon=False,
        fontsize=8.5,
        loc='upper right',
    )
    axes[2].set_ylabel('activation sum', color=INK)
    axes[2].set_xlabel('simulation time / s', color=INK)
    axes[2].set_title(
        'Muscle command and force response',
        loc='left',
        fontsize=11,
        color=INK,
    )

    for axis in axes:
        axis.grid(True, color=GRID, linewidth=0.7)
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
        axis.tick_params(colors=MUTED)
    figure.tight_layout(rect=(0.02, 0.02, 0.98, 0.95))
    figure.savefig(output, dpi=170, bbox_inches='tight')
    plt.close(figure)
    return output


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    outputs = (
        generate_ros_graph(),
        generate_runtime_logic(),
        generate_runtime_signals(),
    )
    for output in outputs:
        print('%s  %d bytes' % (output, output.stat().st_size))


if __name__ == '__main__':
    main()
