"""Range-aided extended Kalman filter for planar localisation.

State is ``[x, y, theta]``. Wheel odometry provides the control input, and each
UWB range is applied as a scalar update after a chi-square innovation gate, so
that non-line-of-sight ranges are rejected instead of dragging the estimate.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .odometry import wrap_angle

EPS = 1e-9


@dataclass
class UpdateOutcome:
    """Per-epoch bookkeeping of the range updates."""

    accepted: int = 0
    rejected: int = 0
    nis: list[float] = field(default_factory=list)
    residuals: list[float] = field(default_factory=list)
    rejected_indices: list[int] = field(default_factory=list)
    status: list[str] = field(default_factory=list)


class RangeEkf:
    """EKF with odometry prediction and gated range updates."""

    def __init__(
        self,
        initial_state,
        initial_covariance,
        sigma_range: float = 0.04,
        sigma_v: float = 0.02,
        sigma_omega: float = 0.03,
        gate_chi2: float = 6.635,
        use_gating: bool = True,
    ) -> None:
        state = np.asarray(initial_state, dtype=float)
        covariance = np.asarray(initial_covariance, dtype=float)
        if state.shape != (3,):
            raise ValueError("initial_state must be (x, y, theta)")
        if covariance.shape != (3, 3):
            raise ValueError("initial_covariance must be 3x3")
        if sigma_range <= 0.0:
            raise ValueError("sigma_range must be positive")
        self.state = state.copy()
        self.covariance = covariance.copy()
        self.sigma_range = float(sigma_range)
        self.sigma_v = float(sigma_v)
        self.sigma_omega = float(sigma_omega)
        self.gate_chi2 = float(gate_chi2)
        self.use_gating = bool(use_gating)

    # ------------------------------------------------------------------ predict
    def predict(self, v: float, omega: float, dt: float) -> None:
        """Propagate the state with a body twist held over ``dt``."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        x, y, theta = self.state
        theta_mid = theta + 0.5 * omega * dt
        self.state = np.array(
            [
                x + v * dt * np.cos(theta_mid),
                y + v * dt * np.sin(theta_mid),
                wrap_angle(theta + omega * dt),
            ]
        )
        jacobian = np.array(
            [
                [1.0, 0.0, -v * dt * np.sin(theta_mid)],
                [0.0, 1.0, v * dt * np.cos(theta_mid)],
                [0.0, 0.0, 1.0],
            ]
        )
        control = np.array(
            [
                [dt * np.cos(theta_mid), 0.0],
                [dt * np.sin(theta_mid), 0.0],
                [0.0, dt],
            ]
        )
        noise = control @ np.diag([self.sigma_v**2, self.sigma_omega**2]) @ control.T
        self.covariance = jacobian @ self.covariance @ jacobian.T + noise
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    # ------------------------------------------------------------------- update
    def update(self, anchors, ranges) -> UpdateOutcome:
        """Apply all available ranges for one epoch, with innovation gating."""
        anchor_array = np.asarray(anchors, dtype=float)
        measurement = np.asarray(ranges, dtype=float)
        if anchor_array.ndim != 2 or anchor_array.shape[1] != 2:
            raise ValueError("anchors must be (A, 2)")
        if measurement.shape != (anchor_array.shape[0],):
            raise ValueError("one range per anchor is required")
        outcome = UpdateOutcome()
        for index, (anchor, measured) in enumerate(zip(anchor_array, measurement)):
            if not np.isfinite(measured):
                outcome.status.append("skipped")
                continue
            delta = self.state[:2] - anchor
            predicted = float(np.linalg.norm(delta))
            if predicted < EPS:
                outcome.status.append("skipped")
                continue
            jacobian = np.array([delta[0] / predicted, delta[1] / predicted, 0.0])
            innovation = float(measured) - predicted
            variance = float(
                jacobian @ self.covariance @ jacobian.T + self.sigma_range**2
            )
            nis = innovation**2 / max(variance, EPS)
            outcome.nis.append(nis)
            outcome.residuals.append(innovation)
            if self.use_gating and nis > self.gate_chi2:
                outcome.rejected += 1
                outcome.rejected_indices.append(index)
                outcome.status.append("rejected")
                continue
            gain = self.covariance @ jacobian / variance
            self.state = self.state + gain * innovation
            self.state[2] = wrap_angle(self.state[2])
            identity = np.eye(3)
            factor = identity - np.outer(gain, jacobian)
            self.covariance = (
                factor @ self.covariance @ factor.T
                + np.outer(gain, gain) * self.sigma_range**2
            )
            self.covariance = 0.5 * (self.covariance + self.covariance.T)
            outcome.accepted += 1
            outcome.status.append("accepted")
        return outcome

    # ------------------------------------------------------------------ metrics
    def nees(self, truth) -> float:
        """Normalised estimation error squared against a ground-truth pose."""
        truth = np.asarray(truth, dtype=float)
        error = self.state - truth
        error[2] = wrap_angle(error[2])
        covariance = self.covariance + np.eye(3) * 1e-12
        return float(error @ np.linalg.solve(covariance, error))

    @property
    def position(self) -> np.ndarray:
        return self.state[:2].copy()

    @property
    def position_covariance(self) -> np.ndarray:
        return self.covariance[:2, :2].copy()
