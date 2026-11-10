"""Two-state tilt Kalman filter: angle plus gyro bias."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import NoiseModel

EPS = 1e-12


@dataclass
class TiltFilter:
    """``theta`` from gyro integration, corrected by an accelerometer tilt."""

    model: NoiseModel
    accel_sigma_deg: float
    state: np.ndarray
    covariance: np.ndarray

    @classmethod
    def create(
        cls,
        model: NoiseModel,
        accel_sigma_deg: float,
        initial_tilt_deg: float = 0.0,
        initial_bias_deg_s: float = 0.0,
        initial_sigma_deg: float = 1.0,
        initial_bias_sigma_deg_s: float = 0.5,
    ) -> "TiltFilter":
        if accel_sigma_deg <= 0.0:
            raise ValueError("accel_sigma_deg must be positive")
        model.validate()
        return cls(
            model=model,
            accel_sigma_deg=float(accel_sigma_deg),
            state=np.array([initial_tilt_deg, initial_bias_deg_s], dtype=float),
            covariance=np.diag(
                [initial_sigma_deg**2, initial_bias_sigma_deg_s**2]
            ),
        )

    def noise_terms(self, dt: float) -> tuple[float, float]:
        """``(q_angle, q_bias)`` for one prediction step of length ``dt``.

        The bias is modelled as constant, so only the angle receives process
        noise; a small bias term keeps the covariance from collapsing.
        """
        return self.model.sample_noise(dt) * dt**2, 1e-12

    def predict(self, gyro_deg_s: float, dt: float) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        q_angle, q_bias = self.noise_terms(dt)
        transition = np.array([[1.0, -dt], [0.0, 1.0]])
        self.state[0] = self.state[0] + (gyro_deg_s - self.state[1]) * dt
        self.covariance = (
            transition @ self.covariance @ transition.T
            + np.array([[q_angle, 0.0], [0.0, q_bias]])
        )

    def update(self, accel_tilt_deg: float) -> float:
        innovation = accel_tilt_deg - self.state[0]
        variance = self.covariance[0, 0] + self.accel_sigma_deg**2
        gain = self.covariance[:, 0] / variance
        self.state = self.state + gain * innovation
        factor = np.eye(2) - np.outer(gain, np.array([1.0, 0.0]))
        self.covariance = (
            factor @ self.covariance @ factor.T
            + np.outer(gain, gain) * self.accel_sigma_deg**2
        )
        return innovation**2 / max(variance, EPS)

    def nees(self, truth_tilt_deg: float, truth_bias_deg_s: float = 0.0) -> float:
        error = np.array(
            [self.state[0] - truth_tilt_deg, self.state[1] - truth_bias_deg_s]
        )
        covariance = self.covariance + np.eye(2) * 1e-12
        return float(error @ np.linalg.solve(covariance, error))


def run_filter(
    sweep: dict, model: NoiseModel, accel_sigma_deg: float | None = None
) -> dict:
    """Run the filter over a simulated sweep and collect errors and NEES."""
    dt = float(sweep["dt"])
    truth = np.asarray(sweep["tilt_deg"], dtype=float)
    gyro = np.asarray(sweep["gyro_deg_s"], dtype=float)
    accel = np.asarray(sweep["accel_tilt_deg"], dtype=float)
    sigma = sweep.get("accel_sigma_deg", 0.35) if accel_sigma_deg is None else accel_sigma_deg
    filter_ = TiltFilter.create(model, sigma, initial_tilt_deg=float(truth[0]))
    estimates = np.empty_like(truth)
    nees = np.empty_like(truth)
    for index in range(len(truth)):
        if index > 0:
            filter_.predict(float(gyro[index - 1]), dt)
        filter_.update(float(accel[index]))
        estimates[index] = filter_.state[0]
        nees[index] = filter_.nees(
            float(truth[index]), float(sweep.get("constant_bias_deg_s", 0.0))
        )
    error = estimates - truth
    return {
        "estimates_deg": estimates,
        "error_deg": error,
        "nees": nees,
        "rmse_deg": float(np.sqrt(np.mean(error**2))),
        "max_error_deg": float(np.max(np.abs(error))),
        "nees_mean": float(np.mean(nees)),
    }
