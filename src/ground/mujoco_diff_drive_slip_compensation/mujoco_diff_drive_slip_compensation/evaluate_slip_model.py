"""Evaluate offline slip compensation on held-out MuJoCo episodes."""

import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np

from .slip_model import TemporalSlipMlp, make_windows, read_episode


def evaluate(data_directory: Path, model_path: Path, output_path: Path) -> dict:
    """Replay only test episodes and aggregate results by friction."""
    episodes = [
        read_episode(path)
        for path in sorted(Path(data_directory).glob('*.csv'))
    ]
    episodes = [episode for episode in episodes if episode.split == 'test']
    if not episodes:
        raise ValueError('no test episodes found')
    model = TemporalSlipMlp.load(model_path)
    by_friction: dict[str, list[float]] = {}
    all_truth: list[float] = []
    all_prediction: list[float] = []
    baseline_error: list[float] = []
    compensated_error: list[float] = []
    latency_ns: list[int] = []
    for episode in episodes:
        windows = make_windows([episode], model.window_size)
        prediction = np.asarray([
            model.predict(window) for window in windows.features
        ])
        truth = windows.targets
        with (Path(data_directory) / f'{episode.episode_id}.csv').open(
            encoding='utf-8', newline=''
        ) as handle:
            rows = list(csv.DictReader(handle))
        endpoint_rows = rows[model.window_size - 1:]
        friction = float(endpoint_rows[0]['friction_coefficient'])
        commands = np.asarray(
            [float(row['command_linear_x']) for row in endpoint_rows]
        )
        actual = np.asarray(
            [float(row['ground_truth_linear_x']) for row in endpoint_rows]
        )
        corrected = commands / np.maximum(1.0 - prediction, 0.5)
        error = np.abs(corrected - actual)
        raw_error = np.abs(commands - actual)
        baseline_error.extend(raw_error.tolist())
        compensated_error.extend(error.tolist())
        all_truth.extend(truth.tolist())
        all_prediction.extend(prediction.tolist())
        friction_key = f'{friction:.3f}'
        by_friction.setdefault(friction_key, []).extend(error.tolist())
        for window in windows.features:
            start = time.perf_counter_ns()
            model.predict(window)
            latency_ns.append(time.perf_counter_ns() - start)
    truth_array = np.asarray(all_truth)
    prediction_array = np.asarray(all_prediction)
    slip_error = prediction_array - truth_array
    result = {
        'episodes': sorted(episode.episode_id for episode in episodes),
        'samples': int(truth_array.size),
        'slip_mae': float(np.mean(np.abs(slip_error))),
        'slip_rmse': float(np.sqrt(np.mean(slip_error ** 2))),
        'raw_velocity_mae': float(np.mean(baseline_error)),
        'compensated_velocity_mae': float(np.mean(compensated_error)),
        'improvement_ratio': float(
            1.0 - np.mean(compensated_error) / max(np.mean(baseline_error), 1e-12)
        ),
        'median_inference_ms': float(np.median(latency_ns) / 1e6),
        'by_friction_compensated_velocity_mae': {
            key: float(np.mean(values)) for key, values in sorted(by_friction.items())
        },
        'evaluation_scope': (
            'held-out test-episode offline replay; compensated commands are '
            'compared with recorded MuJoCo ground-truth velocity'
        ),
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    Path(output_path).write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    return result


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data/sample'))
    parser.add_argument('--model', type=Path, default=Path('data/models/slip-v1/slip_model.json'))
    parser.add_argument('--output', type=Path, default=Path('data/results/slip-evaluation.json'))
    options = parser.parse_args(args)
    print(json.dumps(evaluate(options.data, options.model, options.output), indent=2))


if __name__ == '__main__':
    main()
