"""Thruster configuration matrix and saturation-aware wrench allocation.

An ROV with ``n`` thrusters maps a force vector ``f`` to a body wrench
``tau = T f``.  Allocation is the inverse problem: find ``f`` for a desired
``tau``.  The pseudo-inverse is exact when the demand is inside the feasible
envelope but ignores the per-thruster limits; clipping afterwards distorts the
wrench.  A small box-constrained QP keeps the residual minimal instead.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Thruster:
    """Mounting position and thrust direction in the body frame."""

    name: str
    position: tuple
    direction: tuple
    max_thrust: float = 40.0

    def column(self) -> np.ndarray:
        position = np.asarray(self.position, dtype=float)
        direction = np.asarray(self.direction, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 0.0:
            raise ValueError(f"{self.name}: direction must be non-zero")
        direction = direction / norm
        return np.concatenate([direction, np.cross(position, direction)])


DEFAULT_THRUSTERS = (
    Thruster("bow_port", (0.35, 0.22, 0.0), (0.7071, 0.7071, 0.0)),
    Thruster("bow_starboard", (0.35, -0.22, 0.0), (0.7071, -0.7071, 0.0)),
    Thruster("stern_port", (-0.35, 0.22, 0.0), (-0.7071, 0.7071, 0.0)),
    Thruster("stern_starboard", (-0.35, -0.22, 0.0), (-0.7071, -0.7071, 0.0)),
    Thruster("vertical_fore_port", (0.30, 0.22, 0.0), (0.0, 0.0, 1.0)),
    Thruster("vertical_fore_starboard", (0.30, -0.22, 0.0), (0.0, 0.0, 1.0)),
    Thruster("vertical_aft_port", (-0.30, 0.22, 0.0), (0.0, 0.0, 1.0)),
    Thruster("vertical_aft_starboard", (-0.30, -0.22, 0.0), (0.0, 0.0, 1.0)),
)


def configuration_matrix(thrusters) -> np.ndarray:
    """Stack the thruster columns into a ``6 x n`` matrix."""
    matrix = np.column_stack([thruster.column() for thruster in thrusters])
    if matrix.shape[0] != 6:
        raise ValueError("wrench must have six components")
    return matrix


def limits(thrusters) -> np.ndarray:
    return np.array([thruster.max_thrust for thruster in thrusters], dtype=float)


def solve_box_qp(hessian, gradient, lower, upper, iterations: int = 200) -> np.ndarray:
    """Cyclic coordinate descent for a box-constrained strictly convex QP."""
    hessian = np.asarray(hessian, dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if np.any(lower > upper):
        raise ValueError("lower bound exceeds upper bound")
    diagonal = np.diag(hessian).copy()
    diagonal[diagonal < 1e-12] = 1e-12
    solution = np.clip(np.zeros_like(gradient), lower, upper)
    for _ in range(iterations):
        change = 0.0
        for index in range(solution.size):
            residual = gradient[index] + hessian[index] @ solution
            updated = float(np.clip(solution[index] - residual / diagonal[index],
                                    lower[index], upper[index]))
            change = max(change, abs(updated - solution[index]))
            solution[index] = updated
        if change < 1e-10:
            break
    return solution


def allocate_pseudo_inverse(matrix, demand, max_thrust=None) -> np.ndarray:
    """Unconstrained minimum-norm solution, optionally clipped afterwards."""
    solution = np.linalg.pinv(matrix) @ np.asarray(demand, dtype=float)
    if max_thrust is not None:
        limit = np.asarray(max_thrust, dtype=float)
        solution = np.clip(solution, -limit, limit)
    return solution


def allocate_qp(matrix, demand, max_thrust, weight: float = 1e-6) -> np.ndarray:
    """Minimum wrench error with a small effort penalty and box limits."""
    matrix = np.asarray(matrix, dtype=float)
    demand = np.asarray(demand, dtype=float)
    limit = np.asarray(max_thrust, dtype=float)
    hessian = matrix.T @ matrix + weight * np.eye(matrix.shape[1])
    gradient = -matrix.T @ demand
    return solve_box_qp(hessian, gradient, -limit, limit)


def wrench_of(matrix, forces) -> np.ndarray:
    return np.asarray(matrix, dtype=float) @ np.asarray(forces, dtype=float)


def error_norm(matrix, forces, demand) -> float:
    return float(np.linalg.norm(wrench_of(matrix, forces) - np.asarray(demand, dtype=float)))


def at_limit_ratio(forces, max_thrust) -> float:
    """Fraction of thrusters sitting on the limit (box-constrained solvers)."""
    limit = np.asarray(max_thrust, dtype=float)
    return float(np.mean(np.abs(forces) >= limit - 1e-9))


def overshoot_ratio(forces, max_thrust) -> float:
    """Fraction of thrusters that *exceed* the limit (only unconstrained solvers)."""
    limit = np.asarray(max_thrust, dtype=float)
    return float(np.mean(np.abs(forces) > limit + 1e-9))


def effort(forces) -> float:
    forces = np.asarray(forces, dtype=float)
    return float(np.sum(forces**2))
