"""Nonlinear ellipsoid fit used by the calibration module.

The algebraic quadric fit is a good starting point but is biased by the
``r = -1`` normalisation.  This module refines the parameters by minimising the
geometric residual ``|| W (m - b) || - B`` with a Gauss-Newton loop, where ``W``
is a full 3x3 soft-iron matrix and ``B`` the known field magnitude.
"""

from __future__ import annotations

import numpy as np


def _unpack(parameters):
    offset = parameters[:3]
    matrix = parameters[3:].reshape(3, 3)
    return offset, matrix


def _residual(parameters, data, field_norm):
    offset, matrix = _unpack(parameters)
    corrected = (data - offset) @ matrix.T
    return np.linalg.norm(corrected, axis=1) - field_norm


def refine(data, initial_offset, initial_matrix, field_norm: float,
           iterations: int = 60, step: float = 1e-6) -> tuple:
    """Gauss-Newton refinement of ``(offset, W)`` with a numeric Jacobian."""
    data = np.asarray(data, dtype=float)
    parameters = np.concatenate(
        [np.asarray(initial_offset, dtype=float), np.asarray(initial_matrix, dtype=float).ravel()]
    )
    residual = _residual(parameters, data, field_norm)
    cost = float(residual @ residual)
    damping = 1e-3
    delta = np.zeros_like(parameters)
    for _ in range(iterations):
        jacobian = np.empty((data.shape[0], parameters.size))
        for index in range(parameters.size):
            perturbed = parameters.copy()
            perturbed[index] += step
            jacobian[:, index] = (_residual(perturbed, data, field_norm) - residual) / step
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        improved = False
        for _ in range(10):
            try:
                delta = np.linalg.solve(normal + damping * np.eye(parameters.size), -gradient)
            except np.linalg.LinAlgError:
                damping *= 10.0
                continue
            candidate = parameters + delta
            new_residual = _residual(candidate, data, field_norm)
            new_cost = float(new_residual @ new_residual)
            if new_cost < cost:
                parameters, residual, cost = candidate, new_residual, new_cost
                damping = max(damping / 3.0, 1e-9)
                improved = True
                break
            damping *= 5.0
        if not improved or np.linalg.norm(delta) < 1e-12:
            break
    offset, matrix = _unpack(parameters)
    return offset, matrix, residual
