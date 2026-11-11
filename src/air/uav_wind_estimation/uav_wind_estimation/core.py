"""Wind estimation from the airspeed/groundspeed triangle.

With ground velocity ``v_g``, heading unit vector ``u`` and true airspeed ``V_a``,
the wind is ``w = v_g - V_a u``.  Only ``|V_a|`` is measured by the pitot, so for
a candidate wind the airspeed follows from ``V_a = u . (v_g - w)``; the residual is
the component of ``v_g - w`` perpendicular to ``u``, which must vanish:

``r_i(w) = (I - u_i u_i^T) (v_g,i - w)``

The module solves that two-parameter least-squares problem, reports the Fisher
information (observability), and compares it with the classic "fly reciprocal
headings and average" trick.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: A heading set is accepted when lambda_min >= 1e-3 * lambda_max.
OBSERVABILITY_RATIO = 1e-3


@dataclass(frozen=True)
class WindEstimate:
    """Estimated wind vector with its covariance and residual."""

    wind: np.ndarray
    covariance: np.ndarray
    residual_rms: float
    samples: int

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.wind))

    @property
    def direction_rad(self) -> float:
        return float(np.arctan2(self.wind[1], self.wind[0]))

    def to_dict(self) -> dict:
        return {
            "wind": self.wind.tolist(),
            "speed_m_s": self.speed,
            "direction_deg": float(np.rad2deg(self.direction_rad)),
            "covariance": self.covariance.tolist(),
            "sigma_speed_m_s": float(np.sqrt(max(np.trace(self.covariance), 0.0))),
            "residual_rms": float(self.residual_rms),
            "samples": int(self.samples),
        }


def residual_vector(wind, ground_velocity, headings) -> np.ndarray:
    """Perpendicular residual of the wind triangle for every sample."""
    ground_velocity = np.asarray(ground_velocity, dtype=float)
    headings = np.asarray(headings, dtype=float)
    wind = np.asarray(wind, dtype=float)
    difference = ground_velocity - wind[None, :]
    along = np.sum(difference * headings, axis=1, keepdims=True)
    perpendicular = difference - along * headings
    return perpendicular.ravel()


def airspeed_estimate(wind, ground_velocity, headings) -> np.ndarray:
    """Airspeed implied by a candidate wind."""
    ground_velocity = np.asarray(ground_velocity, dtype=float)
    headings = np.asarray(headings, dtype=float)
    return np.sum((ground_velocity - np.asarray(wind, dtype=float)) * headings, axis=1)


def fisher_information(headings, sigma_m_s: float = 0.2) -> np.ndarray:
    """Information matrix of the wind estimate for a set of headings."""
    headings = np.asarray(headings, dtype=float)
    if headings.ndim != 2 or headings.shape[1] != 2:
        raise ValueError("headings must be (n, 2)")
    if sigma_m_s <= 0.0:
        raise ValueError("sigma must be positive")
    information = np.zeros((2, 2))
    for heading in headings:
        projection = np.eye(2) - np.outer(heading, heading)
        information += projection
    return information / sigma_m_s**2


def estimate_wind(ground_velocity, headings, iterations: int = 30,
                  sigma_m_s: float = 0.2) -> WindEstimate:
    """Gauss-Newton solution of the wind triangle."""
    ground_velocity = np.asarray(ground_velocity, dtype=float)
    headings = np.asarray(headings, dtype=float)
    if ground_velocity.shape != headings.shape or ground_velocity.ndim != 2:
        raise ValueError("ground_velocity and headings must be (n, 2)")
    if ground_velocity.shape[0] < 3:
        raise ValueError("at least three samples are required")
    information = fisher_information(headings, sigma_m_s)
    values = np.linalg.eigvalsh(information)
    if values[0] < 1e-3 * max(values[-1], 1e-12):
        raise ValueError("headings do not observe the wind")
    wind = np.mean(ground_velocity, axis=0)
    for _ in range(iterations):
        residual = residual_vector(wind, ground_velocity, headings).reshape(-1, 2)
        jacobian = np.zeros((residual.size, 2))
        for index, heading in enumerate(headings):
            projection = np.eye(2) - np.outer(heading, heading)
            jacobian[index * 2 : (index + 1) * 2, :] = -projection
        step, *_ = np.linalg.lstsq(jacobian, -residual.ravel(), rcond=None)
        wind = wind + step
        if np.linalg.norm(step) < 1e-10:
            break
    residual = residual_vector(wind, ground_velocity, headings).reshape(-1, 2)
    covariance = np.linalg.inv(information)
    return WindEstimate(
        wind=wind,
        covariance=covariance,
        residual_rms=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        samples=int(ground_velocity.shape[0]),
    )


def reciprocal_estimate(ground_velocity, headings) -> np.ndarray:
    """Classic trick: average the ground velocities of opposite headings."""
    ground_velocity = np.asarray(ground_velocity, dtype=float)
    headings = np.asarray(headings, dtype=float)
    if ground_velocity.shape[0] < 2:
        raise ValueError("need at least two samples")
    best = None
    for first in range(ground_velocity.shape[0]):
        for second in range(first + 1, ground_velocity.shape[0]):
            dot = float(np.dot(headings[first], headings[second]))
            if dot < -0.9 and (best is None or dot < best[0]):
                best = (dot, 0.5 * (ground_velocity[first] + ground_velocity[second]))
    if best is None:
        return np.mean(ground_velocity, axis=0)
    return best[1]


def observability(headings) -> dict:
    """Eigenvalues of the heading information matrix (larger is better)."""
    information = fisher_information(headings, 1.0)
    values = np.linalg.eigvalsh(information)
    return {
        "eigenvalues": [float(value) for value in values],
        "condition": float(values[-1] / max(values[0], 1e-12)),
        "observable": bool(values[0] > OBSERVABILITY_RATIO * values[-1]),
    }
