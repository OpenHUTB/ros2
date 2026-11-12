"""Allan variance and gyro noise-parameter identification.

Datasheets describe a MEMS gyro by angle random walk ``N`` (deg/sqrt(h)) and
bias instability ``B`` (deg/h).  Both are visible in the Allan deviation of a
static log, and both are needed to tune a tilt filter: ``N`` sets the process
noise, ``B`` sets how fast the bias estimate may move.

Rate random walk is deliberately out of scope: on a one-hour log its
contribution stays below the bias-instability floor for a typical MEMS gyro, so
fitting it would only fit noise.  ``tests`` pins the two identifiable terms.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ADEV contribution of a 1/f flicker floor (IEEE Std 952-1997).
BIAS_INSTABILITY_FACTOR = float(np.sqrt(2.0 * np.log(2.0) / np.pi))  # 0.6643
SECONDS_PER_HOUR = 3600.0


@dataclass(frozen=True)
class NoiseModel:
    """Continuous-time gyro noise parameters in datasheet units."""

    arw: float = 0.30                # deg/sqrt(h)
    bias_instability: float = 3.00   # deg/h

    def validate(self) -> None:
        for name, value in (
            ("arw", self.arw),
            ("bias_instability", self.bias_instability),
        ):
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    def adev(self, tau_s) -> np.ndarray:
        """Model Allan deviation in deg/s for cluster times given in seconds."""
        tau_s = np.asarray(tau_s, dtype=float)
        if np.any(tau_s <= 0.0):
            raise ValueError("cluster times must be positive")
        tau_h = tau_s / SECONDS_PER_HOUR
        variance = self.arw**2 / tau_h + (
            BIAS_INSTABILITY_FACTOR * self.bias_instability
        ) ** 2
        return np.sqrt(variance) / SECONDS_PER_HOUR

    @property
    def angle_noise_density(self) -> float:
        """Gyro white-noise density in deg/s/sqrt(Hz)."""
        return self.arw / 60.0

    @property
    def bias_walk_density(self) -> float:
        """Bias random-walk density in deg/s/sqrt(s).

        Bias instability is the bias wander accumulated over one hour, so the
        random-walk density that reproduces it is ``B / (3600 * sqrt(3600))``.
        """
        return self.bias_instability / (SECONDS_PER_HOUR * np.sqrt(SECONDS_PER_HOUR))

    def sample_noise(self, dt: float) -> tuple[float, float]:
        """Per-sample ``(rate_variance, bias_variance)`` at ``dt``.

        ``rate_variance`` is the variance of one gyro sample (deg^2/s^2) and
        ``bias_variance`` the variance the bias gains in one step (deg^2/s^2).
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        return (
            self.angle_noise_density**2 / dt,
            self.bias_walk_density**2 * dt,
        )


def cluster_means(rates, size: int) -> np.ndarray:
    """Sliding-window means over ``size`` consecutive samples (vectorised)."""
    rates = np.asarray(rates, dtype=float)
    if size < 1:
        raise ValueError("cluster size must be >= 1")
    if size > len(rates):
        raise ValueError("cluster size larger than the series")
    cumulative = np.cumsum(np.insert(rates, 0, 0.0))
    return (cumulative[size:] - cumulative[:-size]) / size


def allan_deviation(rates, dt: float, taus=None) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of a rate signal.

    Returns ``(tau_s, adev)`` in seconds and deg/s.  ``taus`` defaults to a
    logarithmic grid of 60 points spanning 4 samples to one ninth of the log.
    """
    rates = np.asarray(rates, dtype=float)
    if len(rates) < 4:
        raise ValueError("at least four samples are required")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    duration = len(rates) * dt
    if taus is None:
        taus = np.logspace(np.log10(4.0 * dt), np.log10(duration / 9.0), 60)
    taus = np.asarray(taus, dtype=float)
    values = np.full(taus.shape, np.nan)
    for index, tau in enumerate(taus):
        size = max(1, int(round(tau / dt)))
        if size > len(rates) // 3:
            continue
        means = cluster_means(rates, size)
        difference = means[size:] - means[:-size]
        if difference.size == 0:
            continue
        values[index] = np.sqrt(np.sum(difference**2) / (2.0 * difference.size))
    valid = np.isfinite(values)
    return taus[valid], values[valid]


def fit_noise_model(taus_s, adev, min_tau_s: float | None = None) -> NoiseModel:
    """Two-term least-squares fit of the Allan curve.

    ``sigma^2 * 3600^2 = N^2 / tau_h + (0.664 B)^2`` is linear in
    ``(N^2, (0.664B)^2)``.  Columns are normalised before ``lstsq`` because
    ``1/tau_h`` spans five orders of magnitude on a one-hour log.
    """
    taus_s = np.asarray(taus_s, dtype=float)
    adev = np.asarray(adev, dtype=float)
    if taus_s.shape != adev.shape:
        raise ValueError("tau and adev must have the same shape")
    tau_h = taus_s / SECONDS_PER_HOUR
    mask = np.isfinite(adev) & (adev > 0.0)
    if min_tau_s is not None:
        mask &= taus_s >= min_tau_s
    if np.count_nonzero(mask) < 2:
        raise ValueError("need at least two valid points")
    design = np.column_stack([1.0 / tau_h[mask], np.ones(np.count_nonzero(mask))])
    scale = np.max(np.abs(design), axis=0)
    target = (adev[mask] * SECONDS_PER_HOUR) ** 2
    coefficients, *_ = np.linalg.lstsq(design / scale, target, rcond=None)
    coefficients = np.maximum(coefficients / scale, 0.0)
    return NoiseModel(
        arw=float(np.sqrt(coefficients[0])),
        bias_instability=float(np.sqrt(coefficients[1]) / BIAS_INSTABILITY_FACTOR),
    )


@dataclass(frozen=True)
class IdentificationReport:
    """Estimated model plus the relative error against a known truth."""

    model: NoiseModel
    truth: NoiseModel | None = None

    def errors(self) -> dict:
        if self.truth is None:
            return {}
        return {
            "arw_pct": 100.0 * (self.model.arw - self.truth.arw) / self.truth.arw,
            "bias_pct": 100.0
            * (self.model.bias_instability - self.truth.bias_instability)
            / self.truth.bias_instability,
        }


def slope(taus_s, adev) -> float:
    """Local log-log slope of the Allan curve (sanity check)."""
    taus_s = np.asarray(taus_s, dtype=float)
    adev = np.asarray(adev, dtype=float)
    if taus_s.size < 2:
        raise ValueError("need at least two points")
    return float(np.polyfit(np.log10(taus_s), np.log10(adev), 1)[0])
