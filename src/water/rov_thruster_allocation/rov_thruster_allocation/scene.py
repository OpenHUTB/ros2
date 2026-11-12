"""Deterministic wrench demands: inside and outside the feasible envelope."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import DEFAULT_THRUSTERS, configuration_matrix, limits


@dataclass(frozen=True)
class DemandConfig:
    """How the test wrenches are generated."""

    inside_samples: int = 400
    outside_samples: int = 200
    outside_scale: float = 1.8
    seed: int = 0

    def validate(self) -> None:
        if self.inside_samples < 1 or self.outside_samples < 1:
            raise ValueError("sample counts must be positive")
        if self.outside_scale <= 1.0:
            raise ValueError("outside_scale must exceed one")


def make_demands(config: DemandConfig, seed: int | None = None) -> dict:
    """Wrench demands generated from a known feasible thrust vector.

    Inside-envelope demands come from ``T f_true`` with ``|f_true| <= f_max``,
    so an exact allocator must reproduce them.  Outside-envelope demands are the
    same wrenches scaled up, which forces saturation.
    """
    config.validate()
    matrix = configuration_matrix(DEFAULT_THRUSTERS)
    limit = limits(DEFAULT_THRUSTERS)
    rng = np.random.default_rng(config.seed if seed is None else seed)
    inside_forces = rng.uniform(-1.0, 1.0, (config.inside_samples, len(limit))) * limit
    inside_wrenches = inside_forces @ matrix.T
    outside_wrenches = inside_wrenches[: config.outside_samples] * config.outside_scale
    return {
        "matrix": matrix,
        "max_thrust": limit,
        "inside_wrenches": inside_wrenches,
        "outside_wrenches": outside_wrenches,
    }
