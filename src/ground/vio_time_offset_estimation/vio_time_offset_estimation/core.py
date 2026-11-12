"""Cross-correlation estimator for the camera-to-IMU time offset.

Visual-inertial fusion assumes the images and the gyro samples are stamped in
the same clock.  A constant offset ``delta`` delays the visual rate against the
gyro by ``delta``; correlating the two signals over a grid of candidate lags
gives a coarse estimate, and a parabola through the peak gives the sub-sample
refinement.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OffsetEstimate:
    """Result of one offset search."""

    offset_s: float
    peak_correlation: float
    peak_z: float
    refined: bool
    samples: int

    def to_dict(self) -> dict:
        return {
            "offset_s": float(self.offset_s),
            "peak_correlation": float(self.peak_correlation),
            "peak_z": float(self.peak_z),
            "refined": bool(self.refined),
            "samples": int(self.samples),
        }


def _normalise(signal: np.ndarray) -> np.ndarray:
    signal = np.asarray(signal, dtype=float)
    signal = signal - np.mean(signal)
    spread = float(np.std(signal))
    if spread <= 0.0:
        raise ValueError("signal must not be constant")
    return (signal - np.mean(signal)) / spread


def cross_correlation(gyro, visual, max_lag: int) -> np.ndarray:
    """Normalised correlation of ``visual`` against ``gyro`` for each lag.

    Positive lag means the visual signal is *late*: ``visual[i]`` matches
    ``gyro[i - lag]``.
    """
    gyro = _normalise(gyro)
    visual = _normalise(visual)
    if gyro.size != visual.size:
        raise ValueError("signals must have the same length")
    if max_lag < 0 or max_lag >= gyro.size:
        raise ValueError("max_lag must be within the signal")
    values = np.empty(2 * max_lag + 1)
    for index, lag in enumerate(range(-max_lag, max_lag + 1)):
        if lag >= 0:
            left = gyro[: gyro.size - lag] if lag > 0 else gyro
            right = visual[lag:]
        else:
            left = gyro[-lag:]
            right = visual[: visual.size + lag]
        values[index] = float(np.mean(left * right)) if left.size > 2 else -1.0
    return values


def _parabolic_peak(values: np.ndarray, index: int) -> float:
    """Sub-sample peak position from three samples around ``index``."""
    if index <= 0 or index >= values.size - 1:
        return 0.0
    left, centre, right = values[index - 1], values[index], values[index + 1]
    denominator = left - 2.0 * centre + right
    if abs(denominator) < 1e-12:
        return 0.0
    return float(np.clip(0.5 * (left - right) / denominator, -0.5, 0.5))


def estimate_offset(gyro, visual, dt: float, max_offset_s: float = 0.08) -> OffsetEstimate:
    """Estimate the visual delay in seconds."""
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    max_lag = int(round(max_offset_s / dt))
    values = cross_correlation(gyro, visual, max_lag)
    index = int(np.argmax(values))
    peak = values[index]
    offset = (index - max_lag) * dt
    refined = False
    if 0 < index < values.size - 1:
        shift = _parabolic_peak(values, index)
        if abs(shift) > 0.0:
            offset += shift * dt
            refined = True
    return OffsetEstimate(
        offset_s=float(offset),
        peak_correlation=float(peak),
        peak_z=float((peak - np.mean(values)) / max(np.std(values), 1e-12)),
        refined=refined,
        samples=int(values.size),
    )


def align(gyro, visual, offset_s: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Shift the visual signal by ``offset_s`` and return the overlapping pair."""
    shift = int(round(offset_s / dt))
    if abs(shift) >= min(gyro.size, visual.size):
        raise ValueError("offset larger than the signals")
    if shift >= 0:
        return np.asarray(gyro[: gyro.size - shift]), np.asarray(visual[shift:])
    return np.asarray(gyro[-shift:]), np.asarray(visual[: visual.size + shift])


def residual_rms(gyro, visual, offset_s: float, dt: float, scale: float = 1.0) -> float:
    """RMS of ``gyro - scale*visual`` after compensating ``offset_s``."""
    aligned_gyro, aligned_visual = align(gyro, visual, offset_s, dt)
    return float(np.sqrt(np.mean((aligned_gyro - scale * aligned_visual) ** 2)))
