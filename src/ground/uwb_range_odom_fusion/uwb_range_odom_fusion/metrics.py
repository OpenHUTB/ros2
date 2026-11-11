"""Error metrics and filter consistency checks."""

from __future__ import annotations

import math

import numpy as np

from .odometry import wrap_angle


def position_errors(estimates, truth) -> np.ndarray:
    """Euclidean position error per epoch."""
    estimates = np.asarray(estimates, dtype=float)
    truth = np.asarray(truth, dtype=float)
    if estimates.shape != truth.shape:
        raise ValueError("estimates and truth must have the same shape")
    return np.linalg.norm(estimates - truth, axis=1)


def heading_errors(headings, truth_headings) -> np.ndarray:
    """Wrapped heading error per epoch."""
    headings = np.asarray(headings, dtype=float)
    truth_headings = np.asarray(truth_headings, dtype=float)
    if headings.shape != truth_headings.shape:
        raise ValueError("headings and truth must have the same shape")
    return np.array(
        [wrap_angle(float(a - b)) for a, b in zip(headings, truth_headings)]
    )


def summarize(errors) -> dict:
    """RMSE, mean, 95th percentile, maximum and final error of a run."""
    values = np.asarray(errors, dtype=float)
    if values.size == 0:
        raise ValueError("no samples")
    return {
        "rmse": float(np.sqrt(np.mean(values**2))),
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
        "final": float(values[-1]),
        "samples": int(values.size),
    }


def chi2_quantile(dof: int, probability: float) -> float:
    """Wilson-Hilferty approximation of the chi-square quantile.

    Accurate to about 1 % for ``dof >= 2`` and avoids a SciPy dependency.
    """
    if dof <= 0:
        raise ValueError("dof must be positive")
    if not 0.0 < probability < 1.0:
        raise ValueError("probability must be in (0, 1)")
    z = _normal_quantile(probability)
    term = 2.0 / (9.0 * dof)
    return float(dof * (1.0 - term + z * math.sqrt(term)) ** 3)


def _normal_quantile(probability: float) -> float:
    """Acklam's rational approximation of the normal quantile."""
    a = (-3.969683028665376e01, 2.209460984245205e02, -2.759285104469687e02,
         1.383577518672690e02, -3.066479806614716e01, 2.506628277459239e00)
    b = (-5.447609879822406e01, 1.615858368580409e02, -1.556989798598866e02,
         6.680131188771972e01, -1.328068155288572e01)
    c = (-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e00,
         -2.549732539343734e00, 4.374664141464968e00, 2.938163982698783e00)
    d = (7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e00,
         3.754408661907416e00)
    low = 0.02425
    high = 1.0 - low
    if probability < low:
        q = math.sqrt(-2.0 * math.log(probability))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / (
            (((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0
        )
    if probability > high:
        q = math.sqrt(-2.0 * math.log(1.0 - probability))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / (
            (((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0
        )
    q = probability - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / (
        ((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0
    )


def nees_consistency(nees) -> dict:
    """Compare the empirical NEES mean with the 95 % two-sided interval."""
    values = np.asarray([value for value in np.asarray(nees, dtype=float) if np.isfinite(value)])
    if values.size == 0:
        raise ValueError("no finite NEES samples")
    dof = 3
    mean = float(np.mean(values))
    spread = 2.0 * dof / values.size
    low = chi2_quantile(dof, 0.025) / 1.0
    high = chi2_quantile(dof, 0.975)
    return {
        "mean": mean,
        "dof": dof,
        "expected_mean": float(dof),
        "interval_low": float(dof - 1.96 * math.sqrt(spread)),
        "interval_high": float(dof + 1.96 * math.sqrt(spread)),
        "quantile_low": low,
        "quantile_high": high,
        "samples": int(values.size),
        "consistent": bool(
            dof - 1.96 * math.sqrt(spread) <= mean <= dof + 1.96 * math.sqrt(spread)
        ),
    }


def gating_summary(accepted, rejected, nlos) -> dict:
    """Count accepted and rejected range updates per line-of-sight state."""
    accepted = np.asarray(accepted, dtype=bool)
    rejected = np.asarray(rejected, dtype=bool)
    nlos = np.asarray(nlos, dtype=bool)
    return {
        "accepted_los": int(np.sum(accepted & ~nlos)),
        "accepted_nlos": int(np.sum(accepted & nlos)),
        "rejected_los": int(np.sum(rejected & ~nlos)),
        "rejected_nlos": int(np.sum(rejected & nlos)),
        "nlos_total": int(np.sum(nlos)),
        "los_total": int(np.sum(accepted & ~nlos) + np.sum(rejected & ~nlos)),
    }
