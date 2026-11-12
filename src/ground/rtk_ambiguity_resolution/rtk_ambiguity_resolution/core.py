"""RTK double-difference model, float solution, LAMBDA decorrelation and fixing.

A rover and a base share the same satellites.  Differencing between satellites
removes the receiver clock, and differencing between receivers removes the
satellite clock, leaving a carrier-phase model that is linear in the baseline
and in the *integer* double-difference ambiguities.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SPEED_OF_LIGHT = 299792458.0
L1_WAVELENGTH_M = 0.190293672798  # GPS L1, 1575.42 MHz


@dataclass(frozen=True)
class Satellite:
    """Satellite direction as seen from the antenna, in degrees."""

    azimuth_deg: float
    elevation_deg: float

    def unit_vector(self) -> np.ndarray:
        """Line-of-sight vector in the local ENU frame."""
        azimuth = np.deg2rad(self.azimuth_deg)
        elevation = np.deg2rad(self.elevation_deg)
        return np.array(
            [
                np.cos(elevation) * np.sin(azimuth),
                np.cos(elevation) * np.cos(azimuth),
                np.sin(elevation),
            ]
        )


def dd_operator(count: int) -> np.ndarray:
    """Single-difference matrix against satellite 0 (shape ``count-1 x count``)."""
    if count < 2:
        raise ValueError("at least two satellites are required")
    operator = np.zeros((count - 1, count))
    operator[:, 0] = -1.0
    operator[np.arange(count - 1), np.arange(1, count)] = 1.0
    return operator


def geometry_matrix(satellites, reference_index: int = 0) -> np.ndarray:
    """Double-difference design matrix for the baseline (``n-1 x 3``)."""
    if reference_index != 0:
        satellites = [satellites[reference_index]] + [
            satellite for index, satellite in enumerate(satellites) if index != reference_index
        ]
    operator = dd_operator(len(satellites))
    return operator @ np.vstack([satellite.unit_vector() for satellite in satellites])


@dataclass(frozen=True)
class FloatSolution:
    """Real-valued baseline and ambiguity estimate with their covariance."""

    baseline: np.ndarray
    ambiguity: np.ndarray
    cov_bb: np.ndarray
    cov_bn: np.ndarray
    cov_nn: np.ndarray
    residual_rms_m: float


@dataclass(frozen=True)
class FixResult:
    """Outcome of integer least squares and of the ratio test."""

    integer_ambiguity: np.ndarray
    baseline: np.ndarray
    ratio: float
    fixed: bool
    candidates: int


def float_solution(
    design: np.ndarray,
    phase_cycles,
    code_m,
    sigma_phase_cycles: float = 0.01,
    sigma_code_m: float = 0.30,
    wavelength: float = L1_WAVELENGTH_M,
    ambiguity_count: int | None = None,
) -> FloatSolution:
    """Weighted least squares for ``[baseline, ambiguity]``.

    Rows are the double differences: phase rows are ``lambda * phase`` and code
    rows are ``code``; both share the same geometry block.
    """
    phase_cycles = np.asarray(phase_cycles, dtype=float)
    code_m = np.asarray(code_m, dtype=float)
    if phase_cycles.shape != code_m.shape:
        raise ValueError("phase and code must have the same shape")
    count = phase_cycles.size
    if design.shape != (count, 3):
        raise ValueError("design must be (n_dd, 3)")
    if sigma_phase_cycles <= 0.0 or sigma_code_m <= 0.0:
        raise ValueError("noise levels must be positive")

    sigma_phase_m = sigma_phase_cycles * wavelength
    ambiguities = count if ambiguity_count is None else int(ambiguity_count)
    if ambiguities < 1 or count % ambiguities:
        raise ValueError("ambiguity_count must divide the row count")
    phase_amb = np.tile(np.eye(ambiguities), (count // ambiguities, 1))
    phase_row = np.hstack([design, wavelength * phase_amb])
    code_row = np.hstack([design, np.zeros((count, ambiguities))])
    model = np.vstack([phase_row, code_row])
    measurement = np.concatenate([wavelength * phase_cycles, code_m])
    weights = np.concatenate(
        [np.full(count, 1.0 / sigma_phase_m**2), np.full(count, 1.0 / sigma_code_m**2)]
    )
    normal = model.T @ (weights[:, None] * model)
    rhs = model.T @ (weights * measurement)
    if np.linalg.cond(normal) > 1e12:
        raise ValueError("geometry is degenerate")
    solution = np.linalg.solve(normal, rhs)
    covariance = np.linalg.inv(normal)
    baseline = solution[:3]
    ambiguity = solution[3:]
    residual = measurement - model @ solution
    rms = float(np.sqrt(np.mean(residual**2)))
    return FloatSolution(
        baseline=baseline,
        ambiguity=ambiguity,
        cov_bb=covariance[:3, :3],
        cov_bn=covariance[:3, 3:],
        cov_nn=covariance[3:, 3:],
        residual_rms_m=rms,
    )


def _ldl(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Unit lower-triangular factor and diagonal: `matrix = L D L^T`."""
    count = matrix.shape[0]
    lower = np.eye(count)
    diagonal = np.zeros(count)
    for column in range(count):
        diagonal[column] = matrix[column, column] - sum(
            lower[column, k] ** 2 * diagonal[k] for k in range(column)
        )
        if diagonal[column] <= 0.0:
            raise ValueError("covariance is not positive definite")
        for row in range(column + 1, count):
            lower[row, column] = (
                matrix[row, column]
                - sum(lower[row, k] * lower[column, k] * diagonal[k] for k in range(column))
            ) / diagonal[column]
    return lower, diagonal


def decorrelate(cov_nn, max_iterations: int = 200) -> np.ndarray:
    """LAMBDA integer decorrelation, returning the unimodular `Z`.

    `z = Z @ N` and `Q_z = Z Q_N Z^T` has a much flatter conditional
    variance profile, which is what makes the integer search cheap.  Each pass
    applies one integer Gauss transformation (multiplier from the LDL^T factor)
    or one permutation; the covariance is then rebuilt from `Z` so rounding
    never accumulates.
    """
    covariance = np.array(cov_nn, dtype=float, copy=True)
    count = covariance.shape[0]
    if covariance.shape != (count, count):
        raise ValueError("covariance must be square")
    transform = np.eye(count, dtype=int)
    for _ in range(max_iterations):
        lower, diagonal = _ldl(covariance)
        applied = False
        for column in range(count - 2, -1, -1):
            for row in range(column + 1, count):
                ratio = lower[row, column]
                if abs(ratio) <= 0.5:
                    continue
                nearest = int(np.rint(ratio))
                transform[row, :] -= nearest * transform[column, :]
                covariance = transform @ np.asarray(cov_nn, dtype=float) @ transform.T
                applied = True
                break
            if applied:
                break
        if applied:
            continue
        order = np.argsort(diagonal)
        if np.array_equal(order, np.arange(count)):
            break
        transform = transform[order, :]
        covariance = transform @ np.asarray(cov_nn, dtype=float) @ transform.T
    if abs(int(round(np.linalg.det(transform.astype(float))))) != 1:
        raise RuntimeError("decorrelation lost unimodularity")
    return transform


def search_integers(cov_z, z_float, radius_sigmas: float = 3.0) -> list[tuple[float, np.ndarray]]:
    """Enumerate integer candidates inside a box, sorted by quadratic cost."""
    cov_z = np.asarray(cov_z, dtype=float)
    z_float = np.asarray(z_float, dtype=float)
    count = z_float.size
    if cov_z.shape != (count, count):
        raise ValueError("covariance shape mismatch")
    sigma = np.sqrt(np.diag(cov_z))
    ranges = [
        np.arange(
            int(np.floor(z_float[index] - radius_sigmas * sigma[index])),
            int(np.ceil(z_float[index] + radius_sigmas * sigma[index])) + 1,
        )
        for index in range(count)
    ]
    grid = np.meshgrid(*ranges, indexing="ij")
    candidates = np.stack([axis.ravel() for axis in grid], axis=1).astype(int)
    residual = candidates - z_float[None, :]
    cost = np.einsum("ij,jk,ik->i", residual, np.linalg.inv(cov_z), residual)
    order = np.argsort(cost)
    return [(float(cost[index]), candidates[index]) for index in order]


def fix(
    solution: FloatSolution,
    transform: np.ndarray,
    ratio_threshold: float = 3.0,
    radius_sigmas: float = 3.0,
) -> FixResult:
    """Integer least squares with the LAMBDA transform and a ratio test."""
    transform = np.asarray(transform, dtype=int)
    cov_z = transform @ solution.cov_nn @ transform.T
    z_float = transform @ solution.ambiguity
    candidates = search_integers(cov_z, z_float, radius_sigmas)
    if len(candidates) < 2:
        return FixResult(
            integer_ambiguity=np.rint(solution.ambiguity).astype(int),
            baseline=solution.baseline,
            ratio=0.0,
            fixed=False,
            candidates=len(candidates),
        )
    best_cost, best_z = candidates[0]
    second_cost, _ = candidates[1]
    ratio = float(second_cost / max(best_cost, 1e-12))
    integer_z = best_z.astype(float)
    integer_ambiguity = np.linalg.solve(transform.astype(float), integer_z)
    integer_ambiguity = np.rint(integer_ambiguity).astype(int)
    correction = solution.cov_bn @ np.linalg.solve(
        solution.cov_nn, solution.ambiguity - integer_ambiguity
    )
    baseline = solution.baseline - correction
    return FixResult(
        integer_ambiguity=integer_ambiguity,
        baseline=baseline,
        ratio=ratio,
        fixed=ratio >= ratio_threshold,
        candidates=len(candidates),
    )
