"""Run every estimator on the same simulated run and collect metrics."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .ekf import RangeEkf
from .geometry import trilaterate
from .metrics import (
    gating_summary,
    heading_errors,
    nees_consistency,
    position_errors,
    summarize,
)
from .odometry import unicycle_step
from .scenario import Simulation


@dataclass
class EstimatorRun:
    """Trajectory and per-epoch statistics of one estimator."""

    name: str
    positions: np.ndarray
    headings: np.ndarray | None
    nees: np.ndarray | None
    accepted: np.ndarray | None
    rejected: np.ndarray | None
    runtime_s: float
    covariance: np.ndarray | None = None

    def metrics(self, simulation: Simulation) -> dict:
        """Summary of position, heading, consistency, gating and runtime."""
        errors = position_errors(self.positions, simulation.truth[:, :2])
        result = {
            "position_m": summarize(errors),
            "runtime_s": self.runtime_s,
            "runtime_per_epoch_us": 1e6 * self.runtime_s / max(1, len(errors)),
        }
        if self.headings is not None:
            heading = summarize(
                np.degrees(np.abs(heading_errors(self.headings, simulation.truth[:, 2])))
            )
            result["heading_deg"] = heading
        if self.nees is not None and np.any(np.isfinite(self.nees)):
            result["nees"] = nees_consistency(self.nees)
        if self.accepted is not None and self.rejected is not None:
            result["gating"] = gating_summary(
                self.accepted, self.rejected, simulation.nlos
            )
        return result


def run_odometry_only(simulation: Simulation) -> EstimatorRun:
    """Pure wheel odometry: the drift baseline."""
    started = time.perf_counter()
    dt = 1.0 / simulation.config.control_hz
    epochs = len(simulation.truth)
    positions = np.empty((epochs, 2), dtype=float)
    headings = np.empty(epochs, dtype=float)
    state = simulation.truth[0].copy()
    for index in range(epochs):
        if index > 0:
            state = np.array(
                unicycle_step(
                    state,
                    simulation.odometry[index - 1, 0],
                    simulation.odometry[index - 1, 1],
                    dt,
                )
            )
        positions[index] = state[:2]
        headings[index] = state[2]
    runtime = time.perf_counter() - started
    return EstimatorRun(
        name="odometry_only",
        positions=positions,
        headings=headings,
        nees=None,
        accepted=None,
        rejected=None,
        runtime_s=runtime,
    )


def run_trilateration(
    simulation: Simulation, robust: bool = True, max_step_m: float | None = 1.0
) -> EstimatorRun:
    """Per-epoch weighted least-squares fix: no motion model and no gating.

    ``max_step_m`` bounds the position jump between two epochs. It is a plain
    sanity check (1 m per 50 ms is already 20 m/s), not a motion model, but it
    keeps a numerically degenerate fix from leaving the map.
    """
    started = time.perf_counter()
    config = simulation.config
    dt = 1.0 / config.control_hz
    epochs = len(simulation.truth)
    anchors = simulation.anchors
    positions = np.empty((epochs, 2), dtype=float)
    state = simulation.truth[0].copy()
    for index in range(epochs):
        if index > 0:
            state = np.array(
                unicycle_step(
                    state,
                    simulation.odometry[index - 1, 0],
                    simulation.odometry[index - 1, 1],
                    dt,
                )
            )
        measurement = simulation.ranges[index]
        valid = np.isfinite(measurement)
        if np.count_nonzero(valid) >= 3:
            result = trilaterate(
                anchors[valid],
                measurement[valid],
                sigma_range=config.range_sigma,
                initial=state[:2],
                robust=robust,
            )
            if result is not None:
                jump = result.position - state[:2]
                length = float(np.linalg.norm(jump))
                if max_step_m is not None and length > max_step_m:
                    state[:2] = state[:2] + jump * (max_step_m / length)
                else:
                    state[:2] = result.position
        positions[index] = state[:2]
    runtime = time.perf_counter() - started
    return EstimatorRun(
        name="trilateration",
        positions=positions,
        headings=None,
        nees=None,
        accepted=None,
        rejected=None,
        runtime_s=runtime,
    )


def run_ekf(
    simulation: Simulation,
    use_gating: bool = True,
    gate_chi2: float | None = None,
    name: str | None = None,
) -> EstimatorRun:
    """Odometry-driven EKF with chi-square gated range updates."""
    started = time.perf_counter()
    config = simulation.config
    dt = 1.0 / config.control_hz
    epochs = len(simulation.truth)
    gate = config.gate_chi2 if gate_chi2 is None else gate_chi2
    filter_ = RangeEkf(
        initial_state=simulation.truth[0],
        initial_covariance=np.diag([0.05**2, 0.05**2, np.deg2rad(1.0) ** 2]),
        sigma_range=config.range_sigma,
        sigma_v=config.process_sigma_v,
        sigma_omega=config.process_sigma_omega,
        gate_chi2=gate,
        use_gating=use_gating,
    )
    positions = np.empty((epochs, 2), dtype=float)
    headings = np.empty(epochs, dtype=float)
    nees = np.full(epochs, np.nan)
    covariance = np.empty((epochs, 3, 3), dtype=float)
    accepted = np.zeros((epochs, len(simulation.anchors)), dtype=bool)
    rejected = np.zeros((epochs, len(simulation.anchors)), dtype=bool)
    for index in range(epochs):
        if index > 0:
            filter_.predict(
                simulation.odometry[index - 1, 0],
                simulation.odometry[index - 1, 1],
                dt,
            )
        measurement = simulation.ranges[index]
        if np.any(np.isfinite(measurement)):
            outcome = filter_.update(simulation.anchors, measurement)
            for anchor_index, status in enumerate(outcome.status):
                accepted[index, anchor_index] = status == "accepted"
                rejected[index, anchor_index] = status == "rejected"
        positions[index] = filter_.position
        headings[index] = filter_.state[2]
        nees[index] = filter_.nees(simulation.truth[index])
        covariance[index] = filter_.covariance
    runtime = time.perf_counter() - started
    return EstimatorRun(
        name=name or ("ekf_gated" if use_gating else "ekf_ungated"),
        positions=positions,
        headings=headings,
        nees=nees,
        accepted=accepted,
        rejected=rejected,
        runtime_s=runtime,
        covariance=covariance,
    )


def default_estimators(simulation: Simulation) -> list[EstimatorRun]:
    """The four estimators compared in the report."""
    return [
        run_odometry_only(simulation),
        run_trilateration(simulation),
        run_ekf(simulation, use_gating=False),
        run_ekf(simulation, use_gating=True),
    ]
