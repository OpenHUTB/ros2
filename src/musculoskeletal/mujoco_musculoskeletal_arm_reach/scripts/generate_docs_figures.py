#!/usr/bin/env python3
"""Generate static documentation figures for the two-joint arm."""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_musculoskeletal_arm_reach.kinematics import (  # noqa: E402
    ArmGeometry,
    forward_kinematics,
    inverse_kinematics,
)


OUTPUT = (
    REPO_ROOT
    / 'docs'
    / 'img'
    / 'musculoskeletal'
    / 'msk_arm_reach'
    / 'workspace_and_muscles.png'
)


def draw_arm(ax, shoulder, elbow, geometry, color):
    x1, z1 = forward_kinematics(shoulder, 0.0, geometry)
    x2, z2 = forward_kinematics(shoulder, elbow, geometry)
    ax.plot((0.0, x1, x2), (0.0, z1, z2), color=color, linewidth=5)
    ax.scatter((0.0, x1, x2), (0.0, z1, z2), color=color, s=35)


def workspace_panel(ax, geometry):
    ax.add_patch(
        Circle(
            (0.0, 0.0),
            geometry.reach_max_m,
            facecolor='#e8f2ff',
            edgecolor='#3167b1',
            linewidth=1.5,
        )
    )
    ax.add_patch(
        Circle(
            (0.0, 0.0),
            geometry.reach_min_m,
            facecolor='white',
            edgecolor='#a8b5c6',
            linewidth=1.2,
        )
    )
    targets = ((0.38, -0.17), (0.30, -0.10), (0.18, -0.32))
    colors = ('#2864dc', '#0f8b72', '#d3562d')
    for target, color in zip(targets, colors):
        shoulder, elbow = inverse_kinematics(*target, geometry)
        draw_arm(ax, shoulder, elbow, geometry, color)
        ax.scatter(*target, s=55, color=color, edgecolor='white', zorder=5)
        ax.annotate(
            '(%.2f, %.2f)' % target,
            target,
            xytext=(8, 8),
            textcoords='offset points',
            fontsize=8,
            color=color,
        )
    ax.set_title('Two-link reachable workspace', fontsize=13, weight='bold')
    ax.set_xlabel('x / m')
    ax.set_ylabel('z / m')
    ax.set_aspect('equal')
    ax.grid(True, color='#dce4ee', linewidth=0.7)


def muscle_panel(ax):
    ax.set_xlim(-0.08, 0.65)
    ax.set_ylim(-0.12, 0.14)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.plot((0.0, 0.30), (0.0, 0.0), color='#786a55', linewidth=14)
    ax.plot((0.30, 0.58), (0.0, 0.0), color='#9a8666', linewidth=12)
    ax.scatter((0.0, 0.30, 0.58), (0.0, 0.0, 0.0), color='#26476e', s=45)
    ax.text(0.0, -0.09, 'shoulder', ha='center', fontsize=9)
    ax.text(0.30, -0.09, 'elbow', ha='center', fontsize=9)
    ax.text(0.58, -0.09, 'end effector', ha='center', fontsize=9)

    paths = (
        ((0.0, 0.05), (0.085, 0.05), '#cd4a42', 'shoulder flexor'),
        ((0.0, -0.05), (0.085, -0.05), '#2f65b0', 'shoulder extensor'),
        ((0.22, 0.04), (0.375, 0.04), '#e07a32', 'elbow flexor'),
        ((0.22, -0.04), (0.375, -0.04), '#2a8b82', 'elbow extensor'),
        ((0.0, 0.09), (0.485, 0.055), '#a33b75', 'biarticular flexor'),
        ((0.0, -0.09), (0.485, -0.055), '#657a2f', 'biarticular extensor'),
    )
    for start, end, color, label in paths:
        ax.annotate(
            '',
            xy=end,
            xytext=start,
            arrowprops=dict(
                arrowstyle='-',
                color=color,
                linewidth=2.3,
                connectionstyle='arc3,rad=0.08',
            ),
        )
        midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        ax.text(
            midpoint[0],
            midpoint[1] + (0.012 if start[1] >= 0 else -0.022),
            label,
            ha='center',
            fontsize=7,
            color=color,
        )
    ax.set_title('Six Hill-muscle routes', fontsize=13, weight='bold')


def main() -> None:
    geometry = ArmGeometry()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(
        1,
        2,
        figsize=(12, 5.2),
        gridspec_kw={'width_ratios': (1.0, 1.2)},
    )
    figure.patch.set_facecolor('white')
    workspace_panel(axes[0], geometry)
    muscle_panel(axes[1])
    figure.suptitle(
        'Musculoskeletal arm: reachable workspace and muscle routing',
        fontsize=15,
        weight='bold',
    )
    figure.tight_layout(rect=(0.02, 0.02, 0.98, 0.93))
    figure.savefig(OUTPUT, dpi=160, bbox_inches='tight')
    plt.close(figure)
    print('%s  %d bytes' % (OUTPUT, OUTPUT.stat().st_size))


if __name__ == '__main__':
    main()
