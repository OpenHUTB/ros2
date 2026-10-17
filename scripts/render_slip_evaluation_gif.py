"""Render measured command and ground-truth velocities from one test episode."""

import argparse
import csv
from pathlib import Path
import sys

import imageio.v2 as imageio
import numpy as np

PACKAGE_ROOT = Path(__file__).parents[1] / 'src' / 'ground' / 'mujoco_diff_drive_slip_compensation'
sys.path.insert(0, str(PACKAGE_ROOT))

from mujoco_diff_drive_slip_compensation.slip_model import (  # noqa: E402
    TemporalSlipMlp,
    load_episodes,
    make_windows,
)


def _line(image: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int]) -> None:
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for t in np.linspace(0.0, 1.0, steps + 1):
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))
            image[max(0, y - 1):y + 2, max(0, x - 1):x + 2] = color


def render(data: Path, model_path: Path, output: Path, frames: int = 24) -> None:
    episodes = [episode for episode in load_episodes(data) if episode.split == 'test']
    if not episodes:
        raise ValueError('no held-out test episode is available')
    episode = episodes[0]
    model = TemporalSlipMlp.load(model_path)
    windows = make_windows([episode], model.window_size)
    predictions = model.predict_batch(windows.features)
    with (data / f'{episode.episode_id}.csv').open(
        encoding='utf-8', newline=''
    ) as handle:
        rows = list(csv.DictReader(handle))[model.window_size - 1:]
    commands = np.asarray([float(row['command_linear_x']) for row in rows])
    actual = np.asarray([float(row['ground_truth_linear_x']) for row in rows])
    compensated = commands / np.maximum(1.0 - predictions, 0.5)
    timestamps = episode.timestamps_ns[model.window_size - 1:]
    seconds = (timestamps - timestamps[0]) / 1e9
    limits = max(0.1, float(np.max(np.abs(np.concatenate((commands, actual, compensated))))))
    output.parent.mkdir(parents=True, exist_ok=True)
    images = []
    for index in range(1, frames + 1):
        image = np.full((240, 360, 3), 248, dtype=np.uint8)
        image[24:220, 30:350] = (255, 255, 255)
        for x in range(30, 351, 40):
            image[218:221, x:x + 1] = (210, 210, 210)
        for y in range(40, 220, 40):
            image[y:y + 1, 30:351] = (225, 225, 225)
        end = max(2, int(len(seconds) * index / frames))
        span = max(float(seconds[-1]), 1e-9)
        series = (
            (actual, (45, 95, 180)),
            (commands, (215, 145, 35)),
            (compensated, (200, 65, 55)),
        )
        for values, color in series:
            points = [
                (
                    30 + int(319 * seconds[i] / span),
                    120 - int(92 * values[i] / limits),
                )
                for i in range(end)
            ]
            _line(image, points, color)
        image[18:22, 45:80] = (45, 95, 180)
        image[18:22, 140:175] = (215, 145, 35)
        image[18:22, 235:270] = (200, 65, 55)
        images.append(image)
    imageio.mimsave(output, images, duration=0.08, loop=0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data/sample'))
    parser.add_argument('--model', type=Path, default=Path('data/models/slip-v1/slip_model.json'))
    parser.add_argument('--output', type=Path, default=Path('docs/img/ground/slip-evaluation.gif'))
    options = parser.parse_args()
    render(options.data, options.model, options.output)
