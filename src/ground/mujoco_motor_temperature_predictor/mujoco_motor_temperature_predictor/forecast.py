"""Time-series temperature regression from a documented CSV schema."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

FEATURE_FIELDS = ("left_temperature_c", "left_current_a", "speed_mps", "ambient_c")


def load_samples(path: str | Path, horizon_s: float = 5.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if horizon_s <= 0:
        raise ValueError("horizon_s must be positive")
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"time_s", *FEATURE_FIELDS}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"CSV must contain: {', '.join(sorted(required))}")
        rows = list(reader)
    times = np.asarray([float(row["time_s"]) for row in rows], dtype=np.float64)
    if np.any(np.diff(times) <= 0):
        raise ValueError("time_s must be strictly increasing")
    if len(rows) < 40:
        raise ValueError("at least 40 time samples are required")
    features_all = np.asarray([[float(row[name]) for name in FEATURE_FIELDS] for row in rows], dtype=np.float64)
    target_indices = np.searchsorted(times, times + horizon_s, side="left")
    valid = target_indices < len(times)
    indices = np.flatnonzero(valid)
    targets = features_all[target_indices[valid], 0]
    return times[indices], features_all[indices], targets


def _design(features: np.ndarray, mean: np.ndarray, scale: np.ndarray) -> np.ndarray:
    normalized = (features - mean) / scale
    return np.column_stack((np.ones(len(normalized)), normalized, normalized[:, 1] ** 2, normalized[:, 0] * normalized[:, 1]))


def fit_and_evaluate(
    path: str | Path,
    *,
    horizon_s: float = 5.0,
    train_fraction: float = 0.8,
) -> dict[str, object]:
    times, features, targets = load_samples(path, horizon_s)
    split = int(len(features) * train_fraction)
    cadence = float(np.median(np.diff(times)))
    purge = int(np.ceil(horizon_s / cadence)) + 1
    train_end = split - purge
    if not 0.5 <= train_fraction < 0.9 or train_end < 20 or len(features) - split < 10:
        raise ValueError("time split must leave at least 20 training and 10 test samples")
    mean = features[:train_end].mean(axis=0)
    scale = np.maximum(features[:train_end].std(axis=0), 1e-9)
    matrix = _design(features[:train_end], mean, scale)
    weights, *_ = np.linalg.lstsq(matrix, targets[:train_end], rcond=None)
    prediction = _design(features[split:], mean, scale) @ weights
    error = prediction - targets[split:]
    return {
        "horizon_s": float(horizon_s),
        "train_samples": int(train_end),
        "test_samples": int(len(error)),
        "rmse_c": float(np.sqrt(np.mean(error**2))),
        "times_s": times[split:].tolist(),
        "actual_c": targets[split:].tolist(),
        "predicted_c": prediction.tolist(),
        "mean": mean.tolist(),
        "scale": scale.tolist(),
        "weights": weights.tolist(),
    }
