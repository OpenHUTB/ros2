"""Composite utility trading coverage-tour cost against information gain.

The original OverFOMO scheme decides *how fast* to fly from a hand-shaped
function ``G(confidence, coverage_ratio)``.  AP-CPP generalises that into a
spatially resolved objective that also decides *where* to fly:

    U(pi) = sum_k gamma^k * [ w_i * IG(pose_k)          # expected information
                            + w_f * Phi(pose_k)          # frontier proximity
                            - w_d * d(pose_{k-1}, pose_k) / L   # travel effort
                            - w_t * |dYaw| / 180         # heading change
                            - w_r * mean coverage(pose_k)       # revisit penalty
                            - w_c * dev(pose_k, corridor) / L ]  # route deviation

``IG`` is the *exact* entropy reduction the belief update will produce, obtained
by replaying :meth:`CoverageGrid.fuse` on a scratch belief, so the planner is
never optimising a proxy that diverges from what actually happens at runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from ap_cpp.grid_model import CoverageGrid
from ap_cpp.pose import Pose, heading_between, wrap_angle_deg
from ap_cpp.sensor import SensorModel

__all__ = ["UtilityWeights", "StepTerms", "PathEvaluation", "UtilityModel"]


@dataclass
class UtilityWeights:
    """Relative weighting of the competing terms in the objective.

    The defaults were tuned so that, on a uniform-prior field, a pure
    boustrophedon sweep and an information-driven sweep score within a few
    percent of each other - i.e. the planner is *free* to deviate only when the
    belief is genuinely non-uniform.
    """

    information: float = 1.0
    """Weight on expected entropy reduction (nats, normalised field)."""

    frontier: float = 0.25
    """Weight on the fraction of the footprint that is still uncertain."""

    travel: float = 0.30
    """Weight on normalised travel effort, ``distance / reference_length``."""

    turn: float = 0.12
    """Weight on normalised heading change."""

    revisit: float = 0.35
    """Weight on the already-accumulated coverage at the observation pose."""

    corridor: float = 1.15
    """Weight on lateral deviation from the reference sweep corridor.

    This is the term that makes AP-CPP a *safe* extension of the published
    method rather than a replacement for it.  On a field where nothing is
    known in particular (a uniform prior), information gain is nearly flat
    over the map, so a purely information-driven objective is indifferent
    between flying the planned lanes and wandering - and wandering loses
    coverage over a long mission because the excursion candidates each
    re-observe ground the sweep already had.

    Charging a cost proportional to how far a candidate strays from the
    reference route restores the sweep as the default behaviour, while still
    letting a genuine information hotspot (whose gain dwarfs the deviation
    cost) pull the vehicle off the lanes.
    """

    discount: float = 0.92
    """Geometric discount per horizon step."""

    assumed_measurement_entropy: float = 0.25
    """Frame entropy the planner expects to see when scoring future views.

    A conservative default: the segmentation head is assumed to be *reasonably*
    but not perfectly confident about unseen terrain.
    """

    frontier_entropy_threshold: float = 0.5
    """Entropy above which a cell counts towards the frontier bonus."""

    def as_dict(self) -> dict:
        return {
            "information": self.information,
            "frontier": self.frontier,
            "travel": self.travel,
            "turn": self.turn,
            "revisit": self.revisit,
            "discount": self.discount,
            "assumed_measurement_entropy": self.assumed_measurement_entropy,
            "frontier_entropy_threshold": self.frontier_entropy_threshold,
        }


@dataclass
class StepTerms:
    """Decomposition of the utility contributed by a single horizon step."""

    index: int
    information_gain: float = 0.0
    frontier_fraction: float = 0.0
    travel_cost: float = 0.0
    turn_cost: float = 0.0
    revisit_cost: float = 0.0
    corridor_cost: float = 0.0
    distance: float = 0.0
    utility: float = 0.0
    discounted_utility: float = 0.0

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "information_gain": self.information_gain,
            "frontier_fraction": self.frontier_fraction,
            "travel_cost": self.travel_cost,
            "turn_cost": self.turn_cost,
            "revisit_cost": self.revisit_cost,
            "corridor_cost": self.corridor_cost,
            "distance": self.distance,
            "utility": self.utility,
            "discounted_utility": self.discounted_utility,
        }


@dataclass
class PathEvaluation:
    """Result of scoring a candidate path."""

    total: float
    terms: List[StepTerms] = field(default_factory=list)
    travel_distance: float = 0.0
    cumulative_information: float = 0.0

    def as_dict(self) -> dict:
        return {
            "total": self.total,
            "travel_distance": self.travel_distance,
            "cumulative_information": self.cumulative_information,
            "steps": [t.as_dict() for t in self.terms],
        }


class UtilityModel:
    """Scores candidate observation tours against the current belief."""

    def __init__(
        self,
        weights: UtilityWeights = None,
        sensor: SensorModel = None,
        reference_length: Optional[float] = None,
    ):
        self.weights = weights or UtilityWeights()
        self.sensor = sensor or SensorModel()
        self.reference_length = reference_length

    # -- helpers ----------------------------------------------------------- #
    def _reference(self, grid: CoverageGrid, horizon: int) -> float:
        """Length scale that makes travel effort dimensionless."""
        if self.reference_length is not None:
            return float(self.reference_length)
        # One raster cell per horizon step is the natural neutral pace.
        return max(self.sensor.config.max_ground_radius, grid.config.resolution * horizon)

    @staticmethod
    def corridor_deviation(
        poses: Sequence[Pose], corridor_xy: Optional[np.ndarray]
    ) -> List[float]:
        """Per-pose lateral distance to the reference route polyline, in metres.

        Vectorised point-to-segment distance; the corridor is passed as an
        ``(N, 2)`` array so the whole horizon costs one NumPy broadcast.
        """
        if corridor_xy is None or len(corridor_xy) < 2 or len(poses) == 0:
            return [0.0] * len(poses)

        a = corridor_xy[:-1]
        b = corridor_xy[1:]
        ab = b - a
        denom = np.einsum("ij,ij->i", ab, ab)
        denom = np.where(denom < 1e-12, 1e-12, denom)

        out: List[float] = []
        for pose in poses:
            p = pose.xy
            ap = p - a
            t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
            closest = a + t[:, None] * ab
            out.append(float(np.min(np.linalg.norm(p - closest, axis=1))))
        return out

    def evaluate_path(
        self,
        grid: CoverageGrid,
        poses: Sequence[Pose],
        start_pose: Optional[Pose] = None,
        reference_length: Optional[float] = None,
        corridor_xy: Optional[np.ndarray] = None,
    ) -> PathEvaluation:
        """Score an ordered sequence of observation poses.

        The belief is rolled forward on a scratch copy, so a pose that has
        already been observed by an earlier step in the same path correctly
        yields less information gain the second time round.

        ``corridor_xy`` is the reference route as an ``(N, 2)`` array; when
        supplied, lateral deviation from it is charged at
        :attr:`UtilityWeights.corridor`.
        """
        w = self.weights
        terms: List[StepTerms] = []
        if len(poses) == 0:
            return PathEvaluation(total=0.0)

        ref = reference_length or self._reference(grid, len(poses))

        # Scratch belief: copy only what the fusion kernel reads.
        scratch_entropy = grid.entropy.copy()
        scratch_coverage = grid.coverage.copy()

        deviations = self.corridor_deviation(poses, corridor_xy)
        deviation_scale = max(ref, 1e-9)

        total = 0.0
        cum_info = 0.0
        travel_distance = 0.0

        prev = start_pose if start_pose is not None else poses[0]
        prev_yaw = start_pose.yaw_deg if start_pose is not None else poses[0].yaw_deg
        prev_xy = np.array([prev.x, prev.y], dtype=float)

        for k, pose in enumerate(poses):
            step = StepTerms(index=k)

            # -- information ------------------------------------------------
            update = self.sensor.predict_update(
                grid,
                pose,
                entropy=scratch_entropy,
                coverage=scratch_coverage,
                assumed_measurement_entropy=w.assumed_measurement_entropy,
            )
            step.information_gain = update["gain"]

            cells = update["cells"]
            if cells:
                ii = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
                jj = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
                step.frontier_fraction = float(
                    (scratch_entropy[ii, jj] >= w.frontier_entropy_threshold).mean()
                )
                step.revisit_cost = float(scratch_coverage[ii, jj].mean())
                scratch_entropy[ii, jj] = update["posterior"]
                scratch_coverage[ii, jj] = update["posterior_coverage"]

            # -- motion ----------------------------------------------------
            xy = np.array([pose.x, pose.y], dtype=float)
            step.distance = float(np.linalg.norm(xy - prev_xy))
            travel_distance += step.distance
            step.travel_cost = step.distance / max(ref, 1e-9)

            heading = heading_between(
                Pose(prev_xy[0], prev_xy[1], prev_yaw), pose
            )
            step.turn_cost = abs(wrap_angle_deg(heading - prev_yaw)) / 180.0
            step.corridor_cost = deviations[k] / deviation_scale

            # -- composite -------------------------------------------------
            step.utility = (
                w.information * step.information_gain
                + w.frontier * step.frontier_fraction
                - w.travel * step.travel_cost
                - w.turn * step.turn_cost
                - w.revisit * step.revisit_cost
                - w.corridor * step.corridor_cost
            )
            step.discounted_utility = (w.discount ** k) * step.utility

            total += step.discounted_utility
            cum_info += step.information_gain
            terms.append(step)

            prev_xy = xy
            prev_yaw = heading

        return PathEvaluation(
            total=float(total),
            terms=terms,
            travel_distance=float(travel_distance),
            cumulative_information=float(cum_info),
        )

    def evaluate_step(
        self,
        grid: CoverageGrid,
        start_pose: Pose,
        goal_pose: Pose,
        reference_length: Optional[float] = None,
    ) -> StepTerms:
        """Score a single move - a cheap surrogate for greedy one-step lookahead."""
        return self.evaluate_path(
            grid,
            [goal_pose],
            start_pose=start_pose,
            reference_length=reference_length,
        ).terms[0]

    # -- speed shaping ----------------------------------------------------- #
    @staticmethod
    def confidence_from_entropy(mean_entropy: float) -> float:
        """Map the belief entropy onto the ``confidence level`` term of ``G(x, y)``.

        This is the bridge back to the published OverFOMO speed controller: a
        low-entropy belief *is* a high-confidence detection.
        """
        return float(np.clip(1.0 - mean_entropy, 0.0, 1.0))

    @staticmethod
    def g_function(confidence: float, coverage_ratio: float) -> float:
        """The ``G(confidence, coverage_ratio)`` shaping function.

        Reproduces ``check_g_func.py`` from the upstream repository verbatim so
        that the AP-CPP speed channel is numerically identical to the baseline.
        """

        def weight_conf(x):
            return 1.6 * x ** 2 - 1.6 * x + 1

        def weight_cov(x):
            return -1.6 * x ** 2 + 1.6 * x

        def conf_to_adj(x):
            return 2 * x - 1

        def cr_to_adj(x):
            if x <= 0.2:
                return -5 * x + 1
            return -1.25 * x + 0.25

        return weight_conf(confidence) * conf_to_adj(confidence) + weight_cov(
            confidence
        ) * cr_to_adj(coverage_ratio)
