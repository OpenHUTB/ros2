"""Deterministic RTK observations for a short baseline.

One epoch of single-frequency RTK is not enough to fix ambiguities: the float
ambiguity sigma is several cycles, so a fix is attempted on a short block of
epochs that share the same integer ambiguities.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .core import L1_WAVELENGTH_M, Satellite, dd_operator, geometry_matrix

# Five satellites: four double differences, a well-conditioned sky.
DEFAULT_SATELLITES = (
    Satellite(20.0, 70.0),
    Satellite(110.0, 50.0),
    Satellite(200.0, 35.0),
    Satellite(280.0, 25.0),
    Satellite(340.0, 15.0),
)


@dataclass(frozen=True)
class RtkConfig:
    """Geometry and noise of one simulated survey."""

    satellites: tuple = field(default_factory=lambda: DEFAULT_SATELLITES)
    baseline_m: tuple = (0.80, 0.60, 0.10)
    sigma_phase_cycles: float = 0.010
    sigma_code_m: float = 0.30
    wavelength: float = L1_WAVELENGTH_M
    epochs: int = 400
    epochs_per_fix: int = 20
    seed: int = 0

    def validate(self) -> None:
        if len(self.satellites) < 4:
            raise ValueError("at least four satellites are required")
        if self.sigma_phase_cycles <= 0.0 or self.sigma_code_m <= 0.0:
            raise ValueError("noise levels must be positive")
        if self.epochs < self.epochs_per_fix:
            raise ValueError("epochs must cover at least one fix window")
        if self.epochs_per_fix < 1:
            raise ValueError("epochs_per_fix must be positive")

    @property
    def attempts(self) -> int:
        return int(self.epochs // self.epochs_per_fix)


def true_ambiguity(config: RtkConfig, seed: int | None = None) -> np.ndarray:
    """Integer double-difference ambiguities of the simulated scene."""
    rng = np.random.default_rng(config.seed if seed is None else seed)
    single = rng.integers(0, 120, size=len(config.satellites))
    return np.rint(dd_operator(len(config.satellites)) @ single).astype(int)


def simulate_epoch(config: RtkConfig, ambiguity: np.ndarray, rng: np.random.Generator) -> dict:
    """One epoch of double-difference phase and code observations."""
    design = geometry_matrix(config.satellites)
    baseline = np.asarray(config.baseline_m, dtype=float)
    range_dd = design @ baseline
    phase = range_dd / config.wavelength + ambiguity
    phase = phase + rng.normal(0.0, config.sigma_phase_cycles, design.shape[0])
    code = range_dd + rng.normal(0.0, config.sigma_code_m, design.shape[0])
    return {
        "design": design,
        "phase_cycles": phase,
        "code_m": code,
        "baseline": baseline,
        "ambiguity": np.asarray(ambiguity, dtype=int),
    }


def simulate(config: RtkConfig) -> list[dict]:
    """A deterministic sequence of epochs sharing one ambiguity set."""
    config.validate()
    rng = np.random.default_rng(config.seed)
    ambiguity = true_ambiguity(config)
    return [simulate_epoch(config, ambiguity, rng) for _ in range(config.epochs)]


def stack_epochs(epochs: list[dict]) -> dict:
    """Stack a block of epochs into one least-squares problem."""
    if not epochs:
        raise ValueError("empty epoch block")
    design = np.vstack([epoch["design"] for epoch in epochs])
    return {
        "design": design,
        "phase_cycles": np.concatenate([epoch["phase_cycles"] for epoch in epochs]),
        "code_m": np.concatenate([epoch["code_m"] for epoch in epochs]),
        "baseline": epochs[0]["baseline"],
        "ambiguity": epochs[0]["ambiguity"],
    }


def blocks(config: RtkConfig, epochs: list[dict]) -> list[dict]:
    """Split a run into fix windows of ``epochs_per_fix``."""
    return [
        stack_epochs(epochs[start : start + config.epochs_per_fix])
        for start in range(0, len(epochs) - config.epochs_per_fix + 1, config.epochs_per_fix)
    ]
