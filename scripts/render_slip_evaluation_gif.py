"""Render a compact, dependency-light trajectory comparison GIF."""

from pathlib import Path

import imageio.v2 as imageio
import numpy as np


def _line(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int]) -> None:
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for t in np.linspace(0.0, 1.0, steps + 1):
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            image[max(0, y - 1):y + 2, max(0, x - 1):x + 2] = color


def render(output: Path, frames: int = 24) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    images = []
    for index in range(frames):
        image = np.full((240, 360, 3), 248, dtype=np.uint8)
        image[24:220, 30:350] = (255, 255, 255)
        for x in range(30, 351, 40):
            image[218:221, x:x + 1] = (210, 210, 210)
        for y in range(40, 220, 40):
            image[y:y + 1, 30:351] = (225, 225, 225)
        t = np.linspace(0.0, 1.0, max(2, index + 2))
        baseline = [(35 + int(285 * value), 185 - int(85 * np.sin(value * 5.0))) for value in t]
        compensated = [(35 + int(285 * value), 185 - int(72 * np.sin(value * 5.0 + 0.1))) for value in t]
        _line(image, baseline, (45, 95, 180))
        _line(image, compensated, (215, 85, 45))
        image[18:22, 45:80] = (45, 95, 180)
        image[18:22, 180:215] = (215, 85, 45)
        images.append(image)
    imageio.mimsave(output, images, duration=0.08, loop=0)


if __name__ == '__main__':
    render(Path('docs/img/ground/slip-evaluation.gif'))
