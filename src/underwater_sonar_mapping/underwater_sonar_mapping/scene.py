"""Deterministic synthetic sonar scene and offline rendering helpers."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path

import matplotlib

if not os.environ.get("DISPLAY") and os.name != "nt":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class SonarScene:
    """Synthetic seabed and obstacle geometry used by the ROS demo."""

    x: np.ndarray
    y: np.ndarray
    surface_x: np.ndarray
    surface_y: np.ndarray
    depth: np.ndarray
    obstacles: tuple[tuple[float, float, float], ...] = (
        (5.0, 5.0, 3.0),
        (12.0, 15.0, 4.0),
        (15.0, 8.0, 3.5),
    )


def create_scene(grid_points: int = 30, extent: float = 20.0) -> SonarScene:
    """Build a reproducible synthetic seabed with three obstacles."""
    if grid_points < 2:
        raise ValueError("grid_points must be at least 2")
    if extent <= 0:
        raise ValueError("extent must be positive")

    x = np.linspace(0.0, extent, grid_points)
    y = np.linspace(0.0, extent, grid_points)
    surface_x, surface_y = np.meshgrid(x, y)
    depth = 10.0 + 1.5 * np.sin(surface_x / 2.0) + 1.5 * np.cos(surface_y / 2.0)
    return SonarScene(x, y, surface_x, surface_y, depth)


def draw_scene(ax, scene: SonarScene):
    """Draw the seabed, obstacles, and a five-ray sonar fan on ``ax``."""
    ax.set_title("Underwater Sonar Mapping with Obstacles")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Depth (m)")
    surface = ax.plot_surface(
        scene.surface_x,
        scene.surface_y,
        scene.depth,
        cmap="ocean",
        alpha=0.65,
        edgecolor="none",
    )

    for obstacle_x, obstacle_y, obstacle_size in scene.obstacles:
        ax.bar3d(
            obstacle_x,
            obstacle_y,
            10.0,
            obstacle_size,
            obstacle_size,
            obstacle_size,
            color="red",
            alpha=0.9,
        )

    origin = (10.0, 10.0, 0.0)
    for index in range(5):
        angle = index * (2.0 * np.pi / 5.0)
        end_x = 10.0 + 8.0 * np.cos(angle)
        end_y = 10.0 + 8.0 * np.sin(angle)
        ax.plot(
            [origin[0], end_x],
            [origin[1], end_y],
            [origin[2], 10.0],
            color="cyan",
            linewidth=1.2,
            alpha=0.85,
        )
    ax.scatter([origin[0]], [origin[1]], [origin[2]], color="yellow", s=150, marker="^")
    ax.view_init(elev=28, azim=-135)
    return surface


def render_scene(output: str | Path, dpi: int = 150) -> Path:
    """Render the synthetic scene to PNG without requiring a display."""
    output_path = Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure = plt.figure(figsize=(12, 9))
    axis = figure.add_subplot(111, projection="3d")
    draw_scene(axis, create_scene())
    figure.tight_layout()
    figure.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(figure)
    return output_path


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Render the synthetic sonar mapping demo")
    parser.add_argument("--output", default="underwater_sonar_mapping.png", help="PNG output path")
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args(argv)
    print(render_scene(args.output, dpi=args.dpi))


if __name__ == "__main__":
    main()