"""Shared, ROS-independent LiDAR scan helpers.

Keeping preprocessing, threshold labelling and low-confidence fallback in
this module lets the offline generator, Gazebo collector and ROS inference
node use exactly the same rules and keeps them testable without ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

CLASS_NAMES = ('near', 'far', 'free')
DEFAULT_SCAN_SIZE = 360
DEFAULT_SCAN_MAX = 30.0
DEFAULT_DANGER_DISTANCE = 1.0
DEFAULT_FAR_DISTANCE = 3.0
DEFAULT_FALLBACK_CONFIDENCE = 0.45


def prepare_scan(
    ranges: Sequence[float],
    scan_size: int = DEFAULT_SCAN_SIZE,
    scan_max: float = DEFAULT_SCAN_MAX,
) -> np.ndarray:
    """Sanitize and resample a LaserScan range vector.

    Invalid values are replaced before interpolation: ``inf`` means no
    return and becomes ``scan_max``; ``nan`` and negative infinity are
    likewise made safe. The result is a contiguous ``float32`` vector of
    exactly ``scan_size`` elements normalised only to the physical range
    (the neural network performs the final 0..1 scaling).
    """
    if scan_size <= 0:
        raise ValueError('scan_size must be positive')
    if scan_max <= 0:
        raise ValueError('scan_max must be positive')

    raw = np.asarray(ranges, dtype=np.float32).reshape(-1)
    raw = np.nan_to_num(
        raw,
        nan=scan_max,
        posinf=scan_max,
        neginf=0.0,
    )
    raw = np.clip(raw, 0.0, scan_max)

    if raw.size == 0:
        return np.full(scan_size, scan_max, dtype=np.float32)
    if raw.size != scan_size:
        source_index = np.arange(raw.size, dtype=np.float64)
        target_index = np.linspace(0.0, raw.size - 1.0, scan_size)
        raw = np.interp(target_index, source_index, raw).astype(np.float32)
    return raw


def label_for_min_range(
    min_range: float,
    danger_distance: float = DEFAULT_DANGER_DISTANCE,
    far_distance: float = DEFAULT_FAR_DISTANCE,
) -> int:
    """Map the closest valid range to the near/far/free class id."""
    if not np.isfinite(min_range) or min_range >= far_distance:
        return 2
    if min_range < danger_distance:
        return 0
    return 1


def rule_label_for_scan(
    ranges: Sequence[float],
    danger_distance: float = DEFAULT_DANGER_DISTANCE,
    far_distance: float = DEFAULT_FAR_DISTANCE,
    scan_max: float = DEFAULT_SCAN_MAX,
) -> int:
    """Derive a class from the minimum sanitized range in a scan."""
    scan = prepare_scan(ranges, len(ranges) or 1, scan_max=scan_max)
    return label_for_min_range(
        float(np.min(scan)),
        danger_distance=danger_distance,
        far_distance=far_distance,
    )


@dataclass(frozen=True)
class Classification:
    """One class prediction and the metadata needed by the ROS node."""

    class_id: int
    confidence: float
    used_fallback: bool

    @property
    def class_name(self) -> str:
        return CLASS_NAMES[self.class_id]


def classify_scan_message(
    message,
    predict_proba: Callable[[np.ndarray], Sequence[float]],
    input_size: int = DEFAULT_SCAN_SIZE,
    danger_distance: float = DEFAULT_DANGER_DISTANCE,
    far_distance: float = DEFAULT_FAR_DISTANCE,
    fallback_confidence: float = DEFAULT_FALLBACK_CONFIDENCE,
    scan_max: float = DEFAULT_SCAN_MAX,
) -> Classification:
    """Classify a LaserScan-like message through a probability callback.

    ``predict_proba`` receives the prepared 1-D scan and must return three
    probabilities in class-id order. Malformed probabilities and low
    confidence predictions use the geometric threshold rule as a safety
    fallback.
    """
    if not 0.0 <= fallback_confidence <= 1.0:
        raise ValueError('fallback_confidence must be in [0, 1]')

    scan = prepare_scan(message.ranges, input_size, scan_max=scan_max)
    probabilities = np.asarray(predict_proba(scan), dtype=np.float64).reshape(-1)
    if probabilities.size != len(CLASS_NAMES):
        raise ValueError(
            'predict_proba must return %d values, got %d'
            % (len(CLASS_NAMES), probabilities.size)
        )

    probabilities = np.nan_to_num(
        probabilities,
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    probabilities = np.clip(probabilities, 0.0, None)
    probability_sum = float(probabilities.sum())
    if probability_sum > 0.0:
        probabilities /= probability_sum
    else:
        probabilities.fill(1.0 / len(CLASS_NAMES))

    class_id = int(np.argmax(probabilities))
    confidence = float(probabilities[class_id])
    used_fallback = confidence < fallback_confidence
    if used_fallback:
        class_id = label_for_min_range(
            float(np.min(scan)),
            danger_distance=danger_distance,
            far_distance=far_distance,
        )

    return Classification(
        class_id=class_id,
        confidence=confidence,
        used_fallback=used_fallback,
    )
