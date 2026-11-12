"""Stribeck joint friction: model, sweep identification and feed-forward compensation.

A harmonically driven joint with a PD controller has to overcome friction at
every zero crossing.  The friction torque is modelled as

``tau_f(v) = tau_c sign(v) + tau_v v + (tau_s - tau_c) sign(v) exp(-|v|/v_s)``

At constant velocity the inertia term vanishes, so a set of sweep speeds gives
``tau_measured = tau_f(v)`` directly.  The Coulomb and viscous terms are linear,
the Stribeck velocity is found with a one-dimensional grid search followed by a
linear least-squares refit.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-9


@dataclass(frozen=True)
class FrictionModel:
    """Coulomb, viscous, static and Stribeck-velocity parameters."""

    coulomb: float = 0.45
    viscous: float = 0.08
    static: float = 0.65
    stribeck_velocity: float = 0.20

    def validate(self) -> None:
        if self.coulomb < 0.0 or self.viscous < 0.0 or self.static < 0.0:
            raise ValueError("friction coefficients must be non-negative")
        if self.static < self.coulomb:
            raise ValueError("static friction must not be below Coulomb friction")
        if self.stribeck_velocity <= 0.0:
            raise ValueError("stribeck velocity must be positive")

    def torque(self, velocity) -> np.ndarray:
        """Friction torque for a velocity array (rad/s)."""
        velocity = np.asarray(velocity, dtype=float)
        sign = np.sign(velocity)
        magnitude = np.abs(velocity)
        stribeck = (self.static - self.coulomb) * np.exp(-magnitude / self.stribeck_velocity)
        return sign * (self.coulomb + stribeck) + self.viscous * velocity

    def to_dict(self) -> dict:
        return {
            "coulomb": float(self.coulomb),
            "viscous": float(self.viscous),
            "static": float(self.static),
            "stribeck_velocity": float(self.stribeck_velocity),
        }


def sweep_torque(model: FrictionModel, speeds) -> np.ndarray:
    """Steady-state torque of a constant-velocity sweep."""
    return model.torque(np.asarray(speeds, dtype=float))


def _linear_fit(speeds, torques, stribeck_velocity: float) -> tuple:
    """Least squares for ``(coulomb, viscous, static)`` at a fixed ``v_s``."""
    speeds = np.asarray(speeds, dtype=float)
    torques = np.asarray(torques, dtype=float)
    sign = np.sign(speeds)
    magnitude = np.abs(speeds)
    decay = sign * np.exp(-magnitude / stribeck_velocity)
    design = np.column_stack([sign - decay, speeds, decay])
    coefficients, *_ = np.linalg.lstsq(design, torques, rcond=None)
    coulomb, viscous, static = coefficients
    coulomb = max(float(coulomb), 0.0)
    static = max(float(static), coulomb)
    residual = design @ np.array([coulomb, viscous, static]) - torques
    return coulomb, float(viscous), static, float(np.sqrt(np.mean(residual**2)))


def identify_friction(speeds, torques, grid=None) -> tuple[FrictionModel, dict]:
    """Identify the four parameters from constant-velocity sweep torques."""
    speeds = np.asarray(speeds, dtype=float)
    torques = np.asarray(torques, dtype=float)
    if speeds.shape != torques.shape or speeds.ndim != 1:
        raise ValueError("speeds and torques must be 1-D and equally sized")
    if speeds.size < 6:
        raise ValueError("at least six sweep points are required")
    if np.any(np.abs(speeds) < EPS):
        raise ValueError("sweep speeds must be non-zero")
    if grid is None:
        grid = np.logspace(np.log10(0.02), np.log10(2.0), 120)
    best = None
    for velocity in grid:
        coulomb, viscous, static, residual = _linear_fit(speeds, torques, float(velocity))
        if best is None or residual < best[0]:
            best = (residual, coulomb, viscous, static, float(velocity))
    residual, coulomb, viscous, static, velocity = best
    model = FrictionModel(coulomb=coulomb, viscous=viscous, static=static,
                          stribeck_velocity=velocity)
    return model, {"residual_rms": float(residual), "grid_points": int(len(grid))}


def torque_error(model: FrictionModel, truth: FrictionModel) -> dict:
    """Relative parameter errors against a known truth."""
    return {
        "coulomb_pct": 100.0 * (model.coulomb - truth.coulomb) / truth.coulomb,
        "viscous_pct": 100.0 * (model.viscous - truth.viscous) / truth.viscous,
        "static_pct": 100.0 * (model.static - truth.static) / truth.static,
        "stribeck_pct": 100.0
        * (model.stribeck_velocity - truth.stribeck_velocity)
        / truth.stribeck_velocity,
    }
