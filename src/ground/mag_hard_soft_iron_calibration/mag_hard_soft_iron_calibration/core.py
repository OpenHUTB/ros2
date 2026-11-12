"""Hard/soft-iron calibration of a magnetometer plus tilt-compensated heading.

A magnetometer measures ``m = A s + b + n`` where ``s`` is the true field, ``b``
the hard-iron offset and ``A`` the soft-iron matrix.  Rotating the sensor traces
an ellipsoid; fitting that ellipsoid recovers ``b`` and ``A`` up to a scale, and
the scale is fixed by the known field magnitude.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ellipsoid import refine


@dataclass(frozen=True)
class CalibrationResult:
    """Offset, soft-iron matrix and field magnitude of a fitted ellipsoid."""

    offset: np.ndarray
    matrix: np.ndarray          # maps raw measurement to a calibrated vector
    field_norm: float
    residual_rms: float
    samples: int

    def apply(self, measurements) -> np.ndarray:
        """Correct raw measurements into the sensor frame, same units."""
        data = np.atleast_2d(np.asarray(measurements, dtype=float))
        corrected = (data - self.offset) @ self.matrix.T
        return corrected

    def to_dict(self) -> dict:
        return {
            "offset": self.offset.tolist(),
            "matrix": self.matrix.tolist(),
            "field_norm": float(self.field_norm),
            "residual_rms": float(self.residual_rms),
            "samples": int(self.samples),
        }


def fit_ellipsoid(measurements, field_norm: float | None = None) -> CalibrationResult:
    """Hard/soft-iron fit: sphere initialisation refined by Gauss-Newton.

    The residual is the geometric distance `|| W (m - b) || - B` with a
    lower-triangular `W` and the known field magnitude `B`.  Starting from a
    sphere fit avoids the bias of the algebraic `r = -1` normalisation.
    """
    data = np.asarray(measurements, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError("measurements must be (n, 3)")
    if data.shape[0] < 20:
        raise ValueError("at least twenty samples are required")
    offset = fit_hard_iron(data)
    shifted = data - offset
    magnitude = (
        float(field_norm)
        if field_norm is not None
        else float(np.mean(np.linalg.norm(shifted, axis=1)))
    )
    if magnitude <= 0.0:
        raise ValueError("field_norm must be positive")
    spread = np.std(shifted, axis=0)
    spread[spread <= 0.0] = 1.0
    matrix = np.diag(1.0 / spread)
    matrix *= magnitude / max(float(np.mean(np.linalg.norm(shifted @ matrix.T, axis=1))), 1e-12)

    refined_offset, refined_matrix, residual = refine(data, offset, matrix, magnitude)
    # |W x| only fixes W up to an orthogonal transform; take the symmetric
    # square root, which is the standard choice (and matches a symmetric
    # soft-iron matrix) so that the corrected heading is unambiguous.
    normal = refined_matrix.T @ refined_matrix
    values, vectors = np.linalg.eigh(normal)
    refined_matrix = vectors @ np.diag(np.sqrt(np.maximum(values, 0.0))) @ vectors.T
    corrected = (data - refined_offset) @ refined_matrix.T
    residual_rms = float(np.sqrt(np.mean(residual**2)))
    return CalibrationResult(
        offset=refined_offset,
        matrix=refined_matrix,
        field_norm=magnitude,
        residual_rms=residual_rms,
        samples=int(data.shape[0]),
    )


def fit_hard_iron(measurements) -> np.ndarray:
    """Sphere fit (hard iron only), kept as a baseline for comparison."""
    data = np.asarray(measurements, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError("measurements must be (n, 3)")
    design = np.column_stack([2.0 * data, np.ones(data.shape[0])])
    target = np.sum(data**2, axis=1)
    coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
    return coefficients[:3]


def tilt_angles(acceleration) -> tuple[np.ndarray, np.ndarray]:
    """Roll and pitch from a gravity measurement (any consistent frame)."""
    data = np.asarray(acceleration, dtype=float)
    norm = np.linalg.norm(data, axis=1)
    if np.any(norm < 1e-9):
        raise ValueError("gravity vector must not be zero")
    ax, ay, az = data[:, 0], data[:, 1], data[:, 2]
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
    return roll, pitch


def magnetic_heading(field, acceleration=None) -> np.ndarray:
    """Compass heading in radians, tilt compensated when gravity is supplied."""
    data = np.asarray(field, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError("field must be (n, 3)")
    if acceleration is None:
        horizontal_x = data[:, 0]
        horizontal_y = data[:, 1]
    else:
        roll, pitch = tilt_angles(acceleration)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        horizontal_x = data[:, 0] * cos_pitch + data[:, 1] * sin_roll * sin_pitch + data[:, 2] * cos_roll * sin_pitch
        horizontal_y = data[:, 1] * cos_roll - data[:, 2] * sin_roll
    return np.arctan2(-horizontal_y, horizontal_x)


@dataclass
class HeadingFilter:
    """Scalar Kalman filter on heading: gyro prediction, magnetometer update."""

    state: float
    variance: float
    gyro_sigma: float
    mag_sigma: float

    @classmethod
    def create(
        cls,
        initial_heading: float,
        initial_sigma: float = np.deg2rad(30.0),
        gyro_sigma: float = np.deg2rad(0.5),
        mag_sigma: float = np.deg2rad(2.0),
    ) -> "HeadingFilter":
        return cls(
            state=float(initial_heading),
            variance=float(initial_sigma**2),
            gyro_sigma=float(gyro_sigma),
            mag_sigma=float(mag_sigma),
        )

    def predict(self, rate_rad_s: float, dt: float) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self.state = float(np.arctan2(np.sin(self.state + rate_rad_s * dt),
                                      np.cos(self.state + rate_rad_s * dt)))
        self.variance += self.gyro_sigma**2 * dt

    def update(self, heading: float) -> float:
        innovation = float(np.arctan2(np.sin(heading - self.state), np.cos(heading - self.state)))
        gain = self.variance / (self.variance + self.mag_sigma**2)
        self.state = float(np.arctan2(np.sin(self.state + gain * innovation),
                                      np.cos(self.state + gain * innovation)))
        self.variance *= 1.0 - gain
        return gain
