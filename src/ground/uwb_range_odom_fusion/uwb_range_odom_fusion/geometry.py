"""Range-only geometry: GDOP, weighted trilateration and its covariance.

All functions are pure NumPy and independent of ROS so that the algorithm can
be tested and benchmarked on any machine.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-9


def as_anchors(anchors) -> np.ndarray:
    """Validate an anchor list and return an ``(A, 2)`` float array."""
    array = np.asarray(anchors, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError("anchors must be a sequence of (x, y) pairs")
    if not np.all(np.isfinite(array)):
        raise ValueError("anchor coordinates must be finite")
    if array.shape[0] < 3:
        raise ValueError("at least three anchors are required for a 2-D fix")
    return array


def ranges_of(position, anchors) -> np.ndarray:
    """Exact Euclidean range from ``position`` to every anchor."""
    position = np.asarray(position, dtype=float)
    if position.shape != (2,):
        raise ValueError("position must be a 2-D point")
    return np.linalg.norm(as_anchors(anchors) - position, axis=1)


def unit_matrix(position, anchors) -> np.ndarray:
    """Jacobian of the range vector with respect to the robot position."""
    position = np.asarray(position, dtype=float)
    delta = position[None, :] - as_anchors(anchors)
    distance = np.linalg.norm(delta, axis=1)
    if np.any(distance < EPS):
        raise ValueError("position coincides with an anchor")
    return delta / distance[:, None]


def gdop(position, anchors) -> float:
    """Geometric dilution of precision for a range-only fix.

    ``gdop = sqrt(trace((H^T H)^-1))`` with ``H`` the range Jacobian, so a
    measured range error of ``sigma`` maps to an expected position error of
    ``sigma * gdop``. Values grow without bound as the robot approaches the
    line through collinear anchors.
    """
    jacobian = unit_matrix(position, anchors)
    information = jacobian.T @ jacobian
    eigenvalues = np.linalg.eigvalsh(information)
    if eigenvalues[0] <= 1e-12:
        return float("inf")
    return float(np.sqrt(np.trace(np.linalg.inv(information))))


def gdop_map(xs, ys, anchors) -> np.ndarray:
    """Evaluate :func:`gdop` on a grid; ``inf`` where geometry is degenerate."""
    anchor_array = as_anchors(anchors)
    grid_x, grid_y = np.meshgrid(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float))
    points = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1)
    delta = points[:, None, :] - anchor_array[None, :, :]
    distance = np.linalg.norm(delta, axis=2)
    valid = np.all(distance > 1e-6, axis=1)
    safe = np.maximum(distance, 1e-6)
    jacobian = delta / safe[:, :, None]
    a = np.sum(jacobian[:, :, 0] * jacobian[:, :, 0], axis=1)
    b = np.sum(jacobian[:, :, 0] * jacobian[:, :, 1], axis=1)
    c = np.sum(jacobian[:, :, 1] * jacobian[:, :, 1], axis=1)
    determinant = a * c - b * b
    trace_inverse = np.where(np.abs(determinant) > 1e-12, (a + c) / determinant, np.inf)
    trace_inverse = np.where(trace_inverse > 0.0, trace_inverse, np.inf)
    result = np.sqrt(trace_inverse)
    result[~valid] = np.inf
    return result.reshape(len(ys), len(xs))


def range_covariance(position, anchors, sigma_range: float) -> np.ndarray:
    """Linearised position covariance of a range-only fix."""
    jacobian = unit_matrix(position, anchors)
    information = jacobian.T @ jacobian / float(sigma_range) ** 2
    return np.linalg.inv(information)


@dataclass(frozen=True)
class TrilaterationResult:
    """Outcome of one weighted least-squares range fix."""

    position: np.ndarray
    covariance: np.ndarray
    iterations: int
    residual_rms: float
    weights: np.ndarray
    converged: bool

    @property
    def position_rms(self) -> float:
        """Expected position error implied by ``covariance``."""
        return float(np.sqrt(max(np.trace(self.covariance), 0.0)))


def _linear_guess(anchors: np.ndarray, ranges: np.ndarray) -> np.ndarray:
    """Closed-form start point by subtracting the last range equation.

    From ``|p - a_i|^2 = z_i^2`` and ``|p - a_ref|^2 = z_ref^2``:

    ``2 p . (a_i - a_ref) = z_i^2 - z_ref^2 + |a_i|^2 - |a_ref|^2``
    """
    reference = anchors[-1]
    matrix = 2.0 * (anchors[:-1] - reference)
    rhs = (
        ranges[:-1] ** 2
        - ranges[-1] ** 2
        + np.sum(anchors[:-1] ** 2, axis=1)
        - float(reference @ reference)
    )
    solution, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
    return solution


def trilaterate(
    anchors,
    ranges,
    sigma_range: float = 0.04,
    initial=None,
    max_iterations: int = 30,
    tolerance: float = 1e-10,
    robust: bool = True,
    huber_k: float = 2.0,
) -> TrilaterationResult | None:
    """Weighted Gauss-Newton range fix with optional Huber reweighting.

    Returns ``None`` when fewer than three ranges are available, because a
    planar position is then not observable.
    """
    anchor_array = as_anchors(anchors)
    measurement = np.asarray(ranges, dtype=float)
    if measurement.shape != (anchor_array.shape[0],):
        raise ValueError("one range per anchor is required")
    if not np.all(np.isfinite(measurement)):
        raise ValueError("ranges must be finite")

    position = (
        np.asarray(initial, dtype=float).copy()
        if initial is not None
        else _linear_guess(anchor_array, measurement)
    )
    if not np.all(np.isfinite(position)):
        return None

    weights = np.ones(anchor_array.shape[0])
    iterations = 0
    converged = False
    jacobian = unit_matrix(position, anchor_array)
    residual = np.linalg.norm(anchor_array - position, axis=1) - measurement

    while iterations < max_iterations:
        iterations += 1
        delta = position[None, :] - anchor_array
        distance = np.linalg.norm(delta, axis=1)
        distance = np.maximum(distance, EPS)
        jacobian = delta / distance[:, None]
        residual = distance - measurement

        if robust:
            scale = max(sigma_range, EPS)
            ratio = np.abs(residual) / (huber_k * scale)
            weights = np.where(ratio <= 1.0, 1.0, 1.0 / np.maximum(ratio, EPS))
        else:
            weights = np.ones_like(residual)

        normal = jacobian.T @ (weights[:, None] * jacobian)
        gradient = jacobian.T @ (weights * residual)
        if np.linalg.cond(normal) > 1e12:
            return None
        step = np.linalg.solve(normal, gradient)
        position = position - step
        if np.linalg.norm(step) < tolerance:
            converged = True
            break

    delta = position[None, :] - anchor_array
    distance = np.maximum(np.linalg.norm(delta, axis=1), EPS)
    jacobian = delta / distance[:, None]
    residual = distance - measurement
    normal = jacobian.T @ (weights[:, None] * jacobian)
    if np.linalg.cond(normal) > 1e12:
        return None
    covariance = np.linalg.inv(normal) * float(sigma_range) ** 2
    return TrilaterationResult(
        position=position,
        covariance=covariance,
        iterations=iterations,
        residual_rms=float(np.sqrt(np.mean(residual**2))),
        weights=weights,
        converged=converged,
    )


def nlos_bias(position, anchor, pillars) -> tuple[float, float]:
    """Simplified non-line-of-sight model for a circular pillar.

    Returns ``(bias_m, penetration_m)``. A range is biased when the segment
    robot-anchor crosses a pillar; the bias grows with the traversed chord so
    that grazing occlusion costs less than passing straight through.
    """
    position = np.asarray(position, dtype=float)
    anchor = np.asarray(anchor, dtype=float)
    best_chord = 0.0
    for px, py, radius in pillars:
        centre = np.array([px, py], dtype=float)
        direction = anchor - position
        length = float(np.linalg.norm(direction))
        if length < EPS:
            continue
        unit = direction / length
        offset = centre - position
        along = float(offset @ unit)
        if along < -radius or along > length + radius:
            continue
        perpendicular = float(np.linalg.norm(offset - along * unit))
        if perpendicular >= radius:
            continue
        half_chord = np.sqrt(max(radius**2 - perpendicular**2, 0.0))
        best_chord = max(best_chord, min(2.0 * half_chord, length))
    if best_chord <= 0.0:
        return 0.0, 0.0
    return 0.30 + 0.90 * best_chord, best_chord


def segment_distance(point_a, point_b, point) -> float:
    """Shortest distance from ``point`` to segment ``point_a``-``point_b``."""
    point_a = np.asarray(point_a, dtype=float)
    point_b = np.asarray(point_b, dtype=float)
    point = np.asarray(point, dtype=float)
    direction = point_b - point_a
    length_sq = float(direction @ direction)
    if length_sq < EPS:
        return float(np.linalg.norm(point - point_a))
    t = float((point - point_a) @ direction) / length_sq
    t = min(1.0, max(0.0, t))
    return float(np.linalg.norm(point - (point_a + t * direction)))
