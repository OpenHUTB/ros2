"""Episode-safe slip learning and compact temporal-window inference."""

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable

import numpy as np

from .dataset import SCHEMA_VERSION


FEATURE_NAMES = (
    'wheel_left_mean',
    'wheel_right_mean',
    'wheel_mean',
    'wheel_right_minus_left',
    'command_linear_x',
    'command_angular_z',
    'imu_acceleration_x',
    'imu_acceleration_y',
    'imu_acceleration_z',
    'imu_angular_velocity_x',
    'imu_angular_velocity_y',
    'imu_angular_velocity_z',
)
WHEEL_VELOCITY_FIELDS = (
    'wheel_velocity_left_front',
    'wheel_velocity_left_rear',
    'wheel_velocity_right_front',
    'wheel_velocity_right_rear',
)
IMU_FIELDS = (
    'imu_acceleration_x',
    'imu_acceleration_y',
    'imu_acceleration_z',
    'imu_angular_velocity_x',
    'imu_angular_velocity_y',
    'imu_angular_velocity_z',
)
MODEL_FORMAT = 'mujoco-slip-mlp-2'


@dataclass(frozen=True)
class Episode:
    """Validated samples from one indivisible data split unit."""

    episode_id: str
    split: str
    timestamps_ns: np.ndarray
    features: np.ndarray
    targets: np.ndarray


@dataclass(frozen=True)
class WindowSet:
    """Temporal windows and endpoint labels, preserving episode identity."""

    features: np.ndarray
    targets: np.ndarray
    episode_ids: tuple[str, ...]


def _finite_float(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def row_features(
    row: dict[str, str], wheel_radius: float = 0.08
) -> tuple[np.ndarray, float]:
    """Build model features and a bounded encoder-vs-truth slip target."""
    if not math.isfinite(wheel_radius) or wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be finite and positive')
    wheels = np.asarray(
        [_finite_float(row, field) for field in WHEEL_VELOCITY_FIELDS],
        dtype=np.float64,
    )
    left_mean = float(np.mean(wheels[:2]))
    right_mean = float(np.mean(wheels[2:]))
    feature_values = (
        left_mean,
        right_mean,
        float(np.mean(wheels)),
        right_mean - left_mean,
        _finite_float(row, 'command_linear_x'),
        _finite_float(row, 'command_angular_z'),
        *(_finite_float(row, field) for field in IMU_FIELDS),
    )
    encoder_linear = float(np.mean(wheels)) * wheel_radius
    truth_linear = _finite_float(row, 'ground_truth_linear_x')
    if abs(encoder_linear) < 0.15:
        slip = 0.0
    else:
        slip = float(np.clip(1.0 - truth_linear / encoder_linear, -0.5, 0.5))
    return np.asarray(feature_values, dtype=np.float64), slip


def read_episode(csv_path: Path, wheel_radius: float = 0.08) -> Episode:
    """Validate one CSV and its manifest before exposing model samples."""
    csv_path = Path(csv_path)
    manifest_path = csv_path.with_suffix('.json')
    if not manifest_path.is_file():
        raise ValueError(f'missing episode manifest: {manifest_path.name}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('unsupported dataset schema version')
    episode_id = str(manifest.get('episode_id', ''))
    split = str(manifest.get('split', ''))
    if split not in {'train', 'validation', 'test'}:
        raise ValueError(f'invalid split in manifest: {split}')

    timestamps: list[int] = []
    features: list[np.ndarray] = []
    targets: list[float] = []
    with csv_path.open(encoding='utf-8', newline='') as handle:
        reader = csv.DictReader(handle)
        required = {
            'timestamp_ns', 'episode_id', 'split', 'friction_coefficient',
            *WHEEL_VELOCITY_FIELDS, *IMU_FIELDS, 'command_linear_x',
            'command_angular_z', 'ground_truth_linear_x',
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f'missing CSV fields: {", ".join(sorted(missing))}')
        for row in reader:
            if row['episode_id'] != episode_id or row['split'] != split:
                raise ValueError('CSV episode or split disagrees with manifest')
            timestamp = int(row['timestamp_ns'])
            if timestamp < 0 or (timestamps and timestamp <= timestamps[-1]):
                raise ValueError(
                    'timestamps must be non-negative and strictly increasing'
                )
            friction = _finite_float(row, 'friction_coefficient')
            if friction <= 0.0:
                raise ValueError('friction_coefficient must be positive')
            feature, target = row_features(row, wheel_radius)
            timestamps.append(timestamp)
            features.append(feature)
            targets.append(target)
    if len(timestamps) != int(manifest.get('rows', -1)):
        raise ValueError('manifest row count does not match CSV')
    if not timestamps:
        raise ValueError('episode contains no samples')
    return Episode(
        episode_id,
        split,
        np.asarray(timestamps, dtype=np.int64),
        np.stack(features),
        np.asarray(targets, dtype=np.float64),
    )


def load_episodes(directory: Path, wheel_radius: float = 0.08) -> list[Episode]:
    """Load complete episodes and reject split leakage or duplicate IDs."""
    directory = Path(directory)
    episodes = [
        read_episode(path, wheel_radius)
        for path in sorted(directory.glob('*.csv'))
    ]
    if not episodes:
        raise ValueError(f'no episode CSV files found in {directory}')
    seen: dict[str, str] = {}
    for episode in episodes:
        prior = seen.get(episode.episode_id)
        if prior is not None:
            if prior != episode.split:
                raise ValueError(
                    f'episode {episode.episode_id} appears in multiple splits'
                )
            raise ValueError(f'duplicate episode ID: {episode.episode_id}')
        seen[episode.episode_id] = episode.split
    return episodes


def make_windows(episodes: Iterable[Episode], window_size: int) -> WindowSet:
    """Create endpoint-labelled windows independently inside each episode."""
    if window_size <= 0:
        raise ValueError('window_size must be positive')
    windows: list[np.ndarray] = []
    targets: list[float] = []
    episode_ids: list[str] = []
    for episode in episodes:
        for end in range(window_size - 1, len(episode.features)):
            start = end - window_size + 1
            windows.append(episode.features[start:end + 1])
            targets.append(float(episode.targets[end]))
            episode_ids.append(episode.episode_id)
    feature_count = len(FEATURE_NAMES)
    if not windows:
        return WindowSet(
            np.empty((0, window_size, feature_count), dtype=np.float64),
            np.empty((0,), dtype=np.float64),
            (),
        )
    return WindowSet(np.stack(windows), np.asarray(targets), tuple(episode_ids))


def _metrics(
    truth: np.ndarray, prediction: np.ndarray, threshold: float
) -> dict[str, float]:
    truth_positive = np.abs(truth) >= threshold
    predicted_positive = np.abs(prediction) >= threshold
    true_positive = int(np.count_nonzero(truth_positive & predicted_positive))
    false_positive = int(np.count_nonzero(~truth_positive & predicted_positive))
    false_negative = int(np.count_nonzero(truth_positive & ~predicted_positive))
    precision = true_positive / max(true_positive + false_positive, 1)
    recall = true_positive / max(true_positive + false_negative, 1)
    f1 = 2.0 * precision * recall / max(precision + recall, 1e-12)
    error = prediction - truth
    return {
        'samples': int(truth.size),
        'mae': float(np.mean(np.abs(error))),
        'rmse': float(np.sqrt(np.mean(error * error))),
        'slip_f1': float(f1),
        'slip_accuracy': float(np.mean(truth_positive == predicted_positive)),
        'slip_threshold': float(threshold),
    }


class TemporalSlipMlp:
    """Small tanh MLP over a fixed, episode-local temporal window."""

    def __init__(
        self,
        window_size: int,
        hidden_size: int,
        mean: np.ndarray,
        scale: np.ndarray,
        w1: np.ndarray,
        b1: np.ndarray,
        w2: np.ndarray,
        b2: np.ndarray,
        w_skip: np.ndarray | None = None,
    ) -> None:
        self.window_size = int(window_size)
        self.hidden_size = int(hidden_size)
        self.mean = np.asarray(mean, dtype=np.float64)
        self.scale = np.asarray(scale, dtype=np.float64)
        self.w1 = np.asarray(w1, dtype=np.float64)
        self.b1 = np.asarray(b1, dtype=np.float64)
        self.w2 = np.asarray(w2, dtype=np.float64)
        self.b2 = np.asarray(b2, dtype=np.float64)
        if w_skip is None:
            w_skip = np.zeros(
                (self.window_size * len(FEATURE_NAMES), 1), dtype=np.float64
            )
        self.w_skip = np.asarray(w_skip, dtype=np.float64)
        expected_input = self.window_size * len(FEATURE_NAMES)
        if (self.mean.shape != (len(FEATURE_NAMES),)
                or self.scale.shape != self.mean.shape):
            raise ValueError('model normalization shape does not match features')
        if self.w1.shape != (expected_input, self.hidden_size):
            raise ValueError('model first layer shape is invalid')
        if (self.b1.shape != (self.hidden_size,)
                or self.w2.shape != (self.hidden_size, 1)):
            raise ValueError('model layer parameter shape is invalid')
        if self.b2.shape != (1,) or np.any(self.scale <= 0.0):
            raise ValueError('model output or normalization scale is invalid')
        if self.w_skip.shape != (expected_input, 1):
            raise ValueError('model skip layer shape is invalid')

    def predict_batch(self, windows: np.ndarray) -> np.ndarray:
        windows = np.asarray(windows, dtype=np.float64)
        expected = (self.window_size, len(FEATURE_NAMES))
        if windows.ndim != 3 or windows.shape[1:] != expected:
            raise ValueError(
                f'windows must have shape (N, {expected[0]}, {expected[1]})'
            )
        if not np.isfinite(windows).all():
            raise ValueError('windows must contain only finite values')
        normalized = (windows - self.mean) / self.scale
        flattened = normalized.reshape(len(windows), -1)
        hidden = np.tanh(flattened @ self.w1 + self.b1)
        output = hidden @ self.w2 + flattened @ self.w_skip + self.b2
        return np.tanh(output).reshape(-1)

    def predict(self, window: np.ndarray) -> float:
        return float(self.predict_batch(np.asarray(window)[None, :, :])[0])

    def to_dict(self, metrics: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = {
            'format': MODEL_FORMAT,
            'feature_names': list(FEATURE_NAMES),
            'window_size': self.window_size,
            'hidden_size': self.hidden_size,
            'mean': self.mean.tolist(),
            'scale': self.scale.tolist(),
            'w1': self.w1.tolist(),
            'b1': self.b1.tolist(),
            'w2': self.w2.tolist(),
            'b2': self.b2.tolist(),
            'w_skip': self.w_skip.tolist(),
        }
        if metrics is not None:
            payload['metrics'] = metrics
        return payload

    def save(self, path: Path, metrics: dict[str, Any] | None = None) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(
            json.dumps(self.to_dict(metrics), indent=2, allow_nan=False) + '\n',
            encoding='utf-8',
        )

    @classmethod
    def load(cls, path: Path) -> 'TemporalSlipMlp':
        payload = json.loads(Path(path).read_text(encoding='utf-8'))
        if payload.get('format') != MODEL_FORMAT:
            raise ValueError('unsupported slip model format')
        if tuple(payload.get('feature_names', ())) != FEATURE_NAMES:
            raise ValueError('model feature order does not match this package')
        return cls(**{
            name: payload[name]
            for name in (
                'window_size', 'hidden_size', 'mean', 'scale',
                'w1', 'b1', 'w2', 'b2', 'w_skip',
            )
        })

    def benchmark_ms(self, window: np.ndarray, repeats: int = 500) -> float:
        if repeats <= 0:
            raise ValueError('repeats must be positive')
        sample = np.asarray(window, dtype=np.float64)[None, :, :]
        self.predict_batch(sample)
        durations = np.empty(repeats, dtype=np.float64)
        for index in range(repeats):
            start = time.perf_counter_ns()
            self.predict_batch(sample)
            durations[index] = time.perf_counter_ns() - start
        return float(np.median(durations) / 1_000_000.0)


def train_model(
    data_directory: Path,
    output_directory: Path,
    window_size: int = 12,
    hidden_size: int = 24,
    epochs: int = 300,
    seed: int = 25012,
    slip_threshold: float = 0.1,
) -> dict[str, Any]:
    """Train with episode-level splits and train-only feature normalization."""
    if (epochs <= 0 or hidden_size <= 0
            or not math.isfinite(slip_threshold) or slip_threshold <= 0):
        raise ValueError('epochs, hidden_size, and slip_threshold must be positive')
    episodes = load_episodes(data_directory)
    by_split = {name: [episode for episode in episodes if episode.split == name]
                for name in ('train', 'validation', 'test')}
    if any(not group for group in by_split.values()):
        raise ValueError('dataset must contain train, validation, and test episodes')
    episode_sets = {name: {episode.episode_id for episode in group}
                    for name, group in by_split.items()}
    if (episode_sets['train'] & episode_sets['validation']
            or episode_sets['train'] & episode_sets['test']
            or episode_sets['validation'] & episode_sets['test']):
        raise ValueError('episode IDs must be isolated across dataset splits')

    windows = {
        name: make_windows(group, window_size)
        for name, group in by_split.items()
    }
    if any(len(part.targets) == 0 for part in windows.values()):
        raise ValueError('every split must have at least one full temporal window')
    train_rows = np.concatenate([episode.features for episode in by_split['train']])
    mean = np.mean(train_rows, axis=0)
    scale = np.std(train_rows, axis=0)
    scale[scale < 1e-8] = 1.0

    rng = np.random.default_rng(seed)
    input_size = window_size * len(FEATURE_NAMES)
    w1 = rng.normal(0.0, math.sqrt(2.0 / input_size), (input_size, hidden_size))
    b1 = np.zeros(hidden_size, dtype=np.float64)
    w2 = rng.normal(0.0, math.sqrt(1.0 / hidden_size), (hidden_size, 1))
    b2 = np.zeros(1, dtype=np.float64)
    train_x = ((windows['train'].features - mean) / scale).reshape(
        len(windows['train'].targets), -1
    )
    train_y = windows['train'].targets.reshape(-1, 1)
    validation_x = (
        (windows['validation'].features - mean) / scale
    ).reshape(len(windows['validation'].targets), -1)
    validation_y = windows['validation'].targets

    zero_baseline_validation = float(np.mean(np.abs(validation_y)))
    train_hidden = np.tanh(train_x @ w1 + b1)
    augmented_train = np.column_stack(
        (train_hidden, train_x, np.ones(len(train_x)))
    )
    ridge = np.eye(augmented_train.shape[1], dtype=np.float64)
    ridge[-1, -1] = 0.0
    solution = np.linalg.solve(
        augmented_train.T @ augmented_train + ridge,
        augmented_train.T @ train_y,
    )
    w2 = solution[:hidden_size].reshape(hidden_size, 1)
    w_skip = solution[hidden_size:-1].reshape(input_size, 1)
    b2 = solution[-1].copy()
    validation_hidden = np.tanh(validation_x @ w1 + b1)
    initial_prediction = np.tanh(
        validation_hidden @ w2 + validation_x @ w_skip + b2
    ).reshape(-1)
    initial_validation_mae = float(
        np.mean(np.abs(initial_prediction - validation_y))
    )
    best_validation = min(zero_baseline_validation, initial_validation_mae)
    best_parameters: tuple[np.ndarray, ...] | None = None
    if initial_validation_mae < zero_baseline_validation:
        best_parameters = (
            w1.copy(), b1.copy(), w2.copy(), b2.copy(), w_skip.copy()
        )
    huber_delta = 0.05
    patience = max(12, epochs // 5)
    stale_epochs = 0
    learning_rate = 0.0005
    batch_size = min(128, max(16, len(train_y)))
    for _epoch in range(epochs):
        order = rng.permutation(len(train_y))
        for start in range(0, len(order), batch_size):
            indices = order[start:start + batch_size]
            x_batch = train_x[indices]
            y_batch = train_y[indices]
            z1 = x_batch @ w1 + b1
            hidden = np.tanh(z1)
            z2 = hidden @ w2 + b2
            prediction = np.tanh(z2 + x_batch @ w_skip)
            error = prediction - y_batch
            d_prediction = np.clip(error, -huber_delta, huber_delta)
            d_prediction /= len(indices)
            d_z2 = d_prediction * (1.0 - prediction * prediction)
            d_w2 = hidden.T @ d_z2
            d_w_skip = x_batch.T @ d_z2
            d_b2 = np.sum(d_z2, axis=0)
            d_hidden = d_z2 @ w2.T
            d_z1 = d_hidden * (1.0 - hidden * hidden)
            d_w1 = x_batch.T @ d_z1
            d_b1 = np.sum(d_z1, axis=0)
            w1 -= learning_rate * d_w1
            b1 -= learning_rate * d_b1
            w2 -= learning_rate * d_w2
            w_skip -= learning_rate * d_w_skip
            b2 -= learning_rate * d_b2

        validation_prediction = np.tanh(
            np.tanh(validation_x @ w1 + b1) @ w2
            + validation_x @ w_skip + b2
        ).reshape(-1)
        validation_mae = float(np.mean(np.abs(validation_prediction - validation_y)))
        if validation_mae < best_validation - 1e-7:
            best_validation = validation_mae
            best_parameters = tuple(
                parameter.copy()
                for parameter in (w1, b1, w2, b2, w_skip)
            )
            stale_epochs = 0
        else:
            stale_epochs += 1
            if stale_epochs >= patience:
                break
    if best_parameters is None:
        w1 = np.zeros((input_size, hidden_size), dtype=np.float64)
        b1 = np.zeros(hidden_size, dtype=np.float64)
        w2 = np.zeros((hidden_size, 1), dtype=np.float64)
        b2 = np.zeros(1, dtype=np.float64)
        w_skip = np.zeros((input_size, 1), dtype=np.float64)
    else:
        w1, b1, w2, b2, w_skip = best_parameters
    model = TemporalSlipMlp(
        window_size, hidden_size, mean, scale, w1, b1, w2, b2, w_skip
    )
    test_truth = windows['test'].targets
    test_prediction = model.predict_batch(windows['test'].features)
    latency_ms = model.benchmark_ms(windows['test'].features[0])
    metrics = _metrics(test_truth, test_prediction, slip_threshold)
    metrics.update({
        'inference_latency_median_ms': latency_ms,
        'validation_mae': best_validation,
        'zero_baseline_validation_mae': zero_baseline_validation,
        'initial_validation_mae': initial_validation_mae,
        'validation_improved': best_parameters is not None,
        'episodes': {name: sorted(ids) for name, ids in episode_sets.items()},
        'window_size': window_size,
        'seed': seed,
    })
    model.save(Path(output_directory) / 'slip_model.json', metrics)
    summary_path = Path(output_directory) / 'training_metrics.json'
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(metrics, indent=2) + '\n', encoding='utf-8')
    return metrics


def main(args: list[str] | None = None) -> None:
    """Train and evaluate a slip model from day-one episode CSV files."""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data/raw'))
    parser.add_argument('--output', type=Path, default=Path('data/models'))
    parser.add_argument('--window-size', type=int, default=12)
    parser.add_argument('--hidden-size', type=int, default=24)
    parser.add_argument('--epochs', type=int, default=300)
    parser.add_argument('--seed', type=int, default=25012)
    options = parser.parse_args(args)
    metrics = train_model(
        options.data,
        options.output,
        window_size=options.window_size,
        hidden_size=options.hidden_size,
        epochs=options.epochs,
        seed=options.seed,
    )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
