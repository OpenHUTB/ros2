"""Deterministic simulation: reference path, pure-pursuit tracking, UWB ranges.

Everything is driven by an explicit ``numpy.random.Generator`` seeded by the
caller, so a given seed always produces the same trajectory, the same encoder
noise and the same range measurements.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .geometry import nlos_bias
from .odometry import WheelParameters, measure_odometry, unicycle_step, wrap_angle

EPS = 1e-9


@dataclass(frozen=True)
class ScenarioConfig:
    """All tunable parameters of one simulated experiment."""

    duration_s: float = 70.0
    control_hz: float = 20.0
    truth_hz: float = 200.0
    range_hz: float = 10.0
    speed: float = 0.45
    lookahead: float = 0.80
    max_turn_rate: float = 0.90
    half_width: float = 4.5
    half_height: float = 2.8
    corner_radius: float = 1.10
    path_spacing: float = 0.02
    range_sigma: float = 0.04
    outlier_probability: float = 0.02
    outlier_low: float = 1.5
    outlier_high: float = 3.0
    gate_chi2: float = 6.635
    process_sigma_v: float = 0.05
    process_sigma_omega: float = 0.07
    anchors: tuple = ((6.0, 4.0), (-6.0, 4.0), (-6.0, -4.0), (6.0, -4.0))
    pillars: tuple = ((-1.2, 1.3, 0.35), (1.6, -1.4, 0.30))
    wheel: WheelParameters = field(default_factory=WheelParameters)

    def validate(self) -> None:
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if self.control_hz <= 0.0 or self.truth_hz < self.control_hz:
            raise ValueError("truth_hz must not be smaller than control_hz")
        if not 0.0 < self.range_hz <= self.control_hz:
            raise ValueError("range_hz must be in (0, control_hz]")
        if abs(self.truth_hz / self.control_hz - round(self.truth_hz / self.control_hz)) > 1e-9:
            raise ValueError("truth_hz must be an integer multiple of control_hz")
        if len(self.anchors) < 3:
            raise ValueError("at least three anchors are required")
        if self.corner_radius >= min(self.half_width, self.half_height):
            raise ValueError("corner_radius is too large for the arena")
        self.wheel.validate()

    @property
    def anchor_array(self) -> np.ndarray:
        return np.asarray(self.anchors, dtype=float)

    def with_anchor_layout(self, anchors) -> "ScenarioConfig":
        return replace(self, anchors=tuple(tuple(map(float, a)) for a in anchors))


class ReferencePath:
    """Closed path sampled at uniform arc length."""

    def __init__(self, points: np.ndarray, spacing: float) -> None:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] < 4:
            raise ValueError("path needs at least four 2-D points")
        if not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])
        self.spacing = float(spacing)
        self.points = points
        segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(segment)])
        self.length = float(self.s[-1])

    def sample(self, arc_length: float) -> np.ndarray:
        """Point at a given arc length (wraps around the closed path)."""
        s = float(arc_length) % self.length
        x = np.interp(s, self.s, self.points[:, 0])
        y = np.interp(s, self.s, self.points[:, 1])
        return np.array([x, y])

    def tangent(self, arc_length: float) -> np.ndarray:
        """Unit tangent at a given arc length."""
        delta = 0.05
        forward = self.sample(arc_length + delta)
        backward = self.sample(arc_length - delta)
        direction = forward - backward
        norm = float(np.linalg.norm(direction))
        return direction / max(norm, EPS)

    def project(self, point, hint_s: float | None = None, window: float = 1.2) -> float:
        """Arc length of the closest path point, globally or near ``hint_s``."""
        point = np.asarray(point, dtype=float)
        if hint_s is None:
            start, stop = 0, len(self.points)
        else:
            low = float(hint_s) - float(window)
            high = float(hint_s) + float(window)
            start = int(np.searchsorted(self.s, low, side="left"))
            stop = int(np.searchsorted(self.s, high, side="right"))
            start = max(0, min(start, len(self.points) - 1))
            stop = max(start + 1, min(stop, len(self.points)))
        candidates = self.points[start:stop]
        distance = np.linalg.norm(candidates - point, axis=1)
        best = start + int(np.argmin(distance))
        return float(self.s[best])


def build_rounded_rectangle(
    half_width: float, half_height: float, radius: float, spacing: float = 0.02
) -> ReferencePath:
    """Closed rounded rectangle: four straights joined by quarter circles."""
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if radius >= min(half_width, half_height):
        raise ValueError("radius must be smaller than both half extents")
    samples = []

    def straight(start, end):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        count = max(2, int(np.ceil(np.linalg.norm(end - start) / spacing)) + 1)
        for t in np.linspace(0.0, 1.0, count, endpoint=False):
            samples.append(start + t * (end - start))

    def arc(centre, angle_start, angle_end):
        centre = np.asarray(centre, dtype=float)
        count = max(2, int(np.ceil(radius * abs(angle_end - angle_start) / spacing)) + 1)
        for angle in np.linspace(angle_start, angle_end, count, endpoint=False):
            samples.append(centre + radius * np.array([np.cos(angle), np.sin(angle)]))

    straight((half_width, -(half_height - radius)), (half_width, half_height - radius))
    arc((half_width - radius, half_height - radius), 0.0, np.pi / 2)
    straight((half_width - radius, half_height), (-(half_width - radius), half_height))
    arc((-(half_width - radius), half_height - radius), np.pi / 2, np.pi)
    straight((-half_width, half_height - radius), (-half_width, -(half_height - radius)))
    arc((-(half_width - radius), -(half_height - radius)), np.pi, 3 * np.pi / 2)
    straight((-(half_width - radius), -half_height), (half_width - radius, -half_height))
    arc((half_width - radius, -(half_height - radius)), 3 * np.pi / 2, 2 * np.pi)
    return ReferencePath(np.asarray(samples), spacing)


def track_path(path: ReferencePath, config: ScenarioConfig):
    """Pure-pursuit tracking of ``path`` at constant speed.

    Returns the control-rate truth poses, the commanded twists and the time
    stamp of every control epoch.
    """
    config.validate()
    dt = 1.0 / config.control_hz
    substeps = int(round(config.truth_hz / config.control_hz))
    dt_truth = dt / substeps
    epochs = int(round(config.duration_s * config.control_hz)) + 1
    truth = np.empty((epochs, 3), dtype=float)
    commands = np.empty((epochs, 2), dtype=float)
    pose = np.array([path.points[0, 0], path.points[0, 1], 0.0])
    tangent = path.tangent(0.0)
    pose[2] = float(np.arctan2(tangent[1], tangent[0]))
    for index in range(epochs):
        truth[index] = pose
        nearest = path.project(pose[:2])
        target = path.sample(nearest + config.lookahead)
        bearing = float(np.arctan2(target[1] - pose[1], target[0] - pose[0]))
        alpha = wrap_angle(bearing - pose[2])
        omega = float(
            np.clip(2.0 * config.speed * np.sin(alpha) / config.lookahead,
                    -config.max_turn_rate, config.max_turn_rate)
        )
        commands[index] = (config.speed, omega)
        for _ in range(substeps):
            pose = np.array(unicycle_step(pose, config.speed, omega, dt_truth))
    return truth, commands


@dataclass(frozen=True)
class Simulation:
    """One fully simulated run, ready to be fed to any estimator."""

    config: ScenarioConfig
    times: np.ndarray
    truth: np.ndarray
    odometry: np.ndarray
    ranges: np.ndarray
    nlos: np.ndarray
    outliers: np.ndarray
    path: ReferencePath

    @property
    def anchors(self) -> np.ndarray:
        return self.config.anchor_array


def simulate(config: ScenarioConfig, seed: int = 0, path: ReferencePath | None = None) -> Simulation:
    """Generate truth, odometry and UWB ranges for one seeded run."""
    config.validate()
    rng = np.random.default_rng(seed)
    if path is None:
        path = build_rounded_rectangle(
            config.half_width, config.half_height, config.corner_radius, config.path_spacing
        )
    truth, commands = track_path(path, config)
    epochs = len(truth)
    dt = 1.0 / config.control_hz
    times = np.arange(epochs) * dt

    odometry = np.empty((epochs, 2), dtype=float)
    for index in range(epochs):
        odometry[index] = measure_odometry(
            commands[index, 0], commands[index, 1], config.wheel, rng
        )

    anchors = config.anchor_array
    ranges = np.full((epochs, len(anchors)), np.nan)
    nlos = np.zeros((epochs, len(anchors)), dtype=bool)
    outliers = np.zeros((epochs, len(anchors)), dtype=bool)
    stride = int(round(config.control_hz / config.range_hz))
    for index in range(0, epochs, stride):
        position = truth[index, :2]
        nominal = np.linalg.norm(anchors - position, axis=1)
        measured = nominal + rng.normal(0.0, config.range_sigma, size=len(anchors))
        for anchor_index, anchor in enumerate(anchors):
            bias, _ = nlos_bias(position, anchor, config.pillars)
            if bias > 0.0:
                measured[anchor_index] += bias
                nlos[index, anchor_index] = True
            if rng.random() < config.outlier_probability:
                measured[anchor_index] += rng.uniform(
                    config.outlier_low, config.outlier_high
                )
                outliers[index, anchor_index] = True
        ranges[index] = measured
    return Simulation(config, times, truth, odometry, ranges, nlos, outliers, path)
