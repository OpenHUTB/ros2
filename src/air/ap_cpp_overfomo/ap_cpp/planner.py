"""Rolling-horizon active perception planner.

At every replanning epoch the planner:

1. Seeds a candidate set with the *reference* boustrophedon route (the
   baseline behaviour), a set of entropy-biased A* excursions towards the
   highest-uncertainty frontier cells, and a short angular fan of straight
   marches.  Seeding with the reference route is what guarantees AP-CPP never
   performs worse than the published method: any deviation has to *earn* its
   place through the utility function.
2. Scores every candidate with :class:`~ap_cpp.utility.UtilityModel`, which
   rolls the belief forward using the exact same fusion kernel as runtime.
3. Commits only the first ``execute_steps`` poses of the winning path, then
   re-plans from the new pose - the classic receding-horizon / MPC pattern.
   This is what makes the planner robust to model error, wind, and to the
   semantic segmentation model being wrong about what it saw.

The planner is deliberately dependency-free (stdlib ``heapq`` + NumPy) so it
can run onboard or in CI without a simulator.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ap_cpp.grid_model import CoverageGrid
from ap_cpp.pose import Pose, heading_between, wrap_angle_deg
from ap_cpp.sensor import SensorModel
from ap_cpp.utility import PathEvaluation, StepTerms, UtilityModel, UtilityWeights

__all__ = [
    "PlannerConfig",
    "PathCandidate",
    "PlanResult",
    "StepDiagnostics",
    "RollingHorizonPlanner",
]

Cell = Tuple[int, int]


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class PlannerConfig:
    """Tunables for the receding-horizon search."""

    horizon: int = 6
    """Number of observation poses scored per candidate path."""

    execute_steps: int = 2
    """Poses committed to the controller before re-planning."""

    step_length: float = 6.0
    """Nominal spacing between consecutive observation poses, in metres."""

    replan_interval: float = 2.0
    """Minimum wall-clock interval between replans, in seconds."""

    min_replan_distance: float = 1.0
    """Minimum travel before another replan is triggered, in metres."""

    frontier_targets: int = 3
    """Number of uncertainty-frontier cells A* is run towards."""

    fan_offsets_deg: Tuple[int, ...] = (-60, -30, 0, 30, 60)
    """Angular fan used to generate short exploratory candidates."""

    entropy_bias: float = 0.55
    """Strength of the entropy discount in the A* edge cost, in ``[0, 1)``.

    At ``0`` the search is a plain shortest path; higher values make the search
    willing to detour through unexplored cells.
    """

    max_candidates: int = 12
    """Hard cap on the number of candidates evaluated per replan."""

    astar_max_expansions: int = 200_000
    """Safety valve so a pathological raster can never hang the mission."""

    repair_deficit_threshold: float = 0.35
    """Coverage deficit above which a cell is considered a hole worth patching."""

    enable_coverage_repair: bool = True
    """Whether the coverage-repair generator contributes candidates.

    Holes in the coverage map are a *consequence* of leaving the reference
    corridor, so this generator only earns its keep in the active
    configuration.  Switched off, a mission that has flown its route holds
    station instead of being kept alive patching holes it was never pushed
    into - which is what a genuine reference-only baseline must be allowed
    to do, otherwise the ablation silently credits the baseline with work
    the reference sweep never performs.
    """

    reference_speed: float = 3.0
    """Nominal mission speed in m/s, used to time-stamp predicted observations."""

    def as_dict(self) -> dict:
        d = dict(self.__dict__)
        d["fan_offsets_deg"] = list(self.fan_offsets_deg)
        return d


@dataclass
class PathCandidate:
    """A scored candidate path, retained for diagnostics."""

    name: str
    poses: List[Pose]
    evaluation: PathEvaluation

    @property
    def score(self) -> float:
        return self.evaluation.total

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "score": self.score,
            "travel_distance": self.evaluation.travel_distance,
            "cumulative_information": self.evaluation.cumulative_information,
            "steps": len(self.poses),
        }


@dataclass
class PlanResult:
    """Output of a single replanning epoch."""

    poses: List[Pose]
    evaluation: PathEvaluation
    operator: str
    candidates: List[PathCandidate] = field(default_factory=list)
    entropy_before: float = 0.0
    entropy_after: float = 0.0
    planning_time_s: float = 0.0
    frontier_cells: int = 0

    @property
    def predicted_information_gain(self) -> float:
        return self.evaluation.cumulative_information

    @property
    def predicted_entropy_reduction(self) -> float:
        return max(self.entropy_before - self.entropy_after, 0.0)

    def ranking(self) -> List[Tuple[str, float]]:
        return sorted(
            ((c.name, c.score) for c in self.candidates), key=lambda kv: -kv[1]
        )

    def as_dict(self) -> dict:
        return {
            "operator": self.operator,
            "poses": [p.as_tuple() for p in self.poses],
            "score": self.evaluation.total,
            "travel_distance": self.evaluation.travel_distance,
            "predicted_information_gain": self.predicted_information_gain,
            "entropy_before": self.entropy_before,
            "entropy_after": self.entropy_after,
            "planning_time_s": self.planning_time_s,
            "frontier_cells": self.frontier_cells,
            "candidates": [c.as_dict() for c in self.candidates],
        }


@dataclass
class StepDiagnostics:
    """Book-keeping for one executed observation step."""

    index: int
    operator: str
    pose: Pose
    entropy_before: float
    entropy_after: float
    coverage_fraction: float
    information_gain: float
    speed: float
    travelled: float
    candidate_scores: Dict[str, float] = field(default_factory=dict)
    planned_pose_count: int = 0

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "operator": self.operator,
            "x": self.pose.x,
            "y": self.pose.y,
            "yaw_deg": self.pose.yaw_deg,
            "entropy_before": self.entropy_before,
            "entropy_after": self.entropy_after,
            "coverage_fraction": self.coverage_fraction,
            "information_gain": self.information_gain,
            "speed": self.speed,
            "travelled": self.travelled,
            "planned_pose_count": self.planned_pose_count,
            "candidate_scores": dict(self.candidate_scores),
        }


# --------------------------------------------------------------------------- #
# Planner
# --------------------------------------------------------------------------- #
class RollingHorizonPlanner:
    """Receding-horizon planner over the belief grid."""

    def __init__(
        self,
        grid: CoverageGrid,
        sensor: SensorModel = None,
        utility: UtilityModel = None,
        config: PlannerConfig = None,
    ):
        self.grid = grid
        self.sensor = sensor or SensorModel()
        self.config = config or PlannerConfig()
        if utility is None:
            utility = UtilityModel(
                weights=UtilityWeights(),
                sensor=self.sensor,
                reference_length=self.config.step_length,
            )
        self.utility = utility

        self._last_plan_time: float = -math.inf
        self._last_plan_pose: Optional[np.ndarray] = None
        self.plans: int = 0

    # -- public API -------------------------------------------------------- #
    def should_replan(self, pose: Pose, now: float) -> bool:
        """Replan gate: periodic, plus a travel-triggered refresh."""
        if self._last_plan_pose is None:
            return True
        if now - self._last_plan_time >= self.config.replan_interval:
            return True
        return bool(
            np.linalg.norm(pose.xy - self._last_plan_pose)
            >= self.config.min_replan_distance
            and now - self._last_plan_time >= 0.25 * self.config.replan_interval
        )

    def plan(
        self,
        pose: Pose,
        reference_route: Optional[Sequence[Pose]] = None,
        now: float = 0.0,
    ) -> PlanResult:
        """Run one replanning epoch from ``pose``.

        Parameters
        ----------
        pose:
            Current vehicle pose in NED.
        reference_route:
            The pre-computed boustrophedon route.  Used both as a candidate
            (the baseline behaviour) and as the fallback when the grid has
            nothing left to explore.
        now:
            Monotonic mission time, used for the replan gate and for
            ``last_seen`` stamping.
        """
        t0 = time.perf_counter()
        cfg = self.config
        grid = self.grid

        self._last_plan_time = now
        self._last_plan_pose = pose.xy.copy()
        self.plans += 1

        entropy_before = grid.mean_entropy()
        frontier = grid.frontier_cells()
        frontier_count = int(frontier.shape[0])

        candidates = self._build_candidates(pose, reference_route, frontier)

        best: Optional[PathCandidate] = None
        for cand in candidates:
            if best is None or cand.score > best.score:
                best = cand

        if best is None or not best.poses:
            # Nothing to score - hold station, which the mission loop reads as
            # "mission complete".
            return PlanResult(
                poses=[],
                evaluation=PathEvaluation(total=0.0),
                operator="idle",
                candidates=candidates,
                entropy_before=entropy_before,
                entropy_after=entropy_before,
                planning_time_s=time.perf_counter() - t0,
                frontier_cells=frontier_count,
            )

        committed = best.poses[: max(1, cfg.execute_steps)]

        # Predicted belief after executing the committed prefix.
        scratch_h = grid.entropy.copy()
        scratch_c = grid.coverage.copy()
        for p in committed:
            upd = self.sensor.predict_update(
                grid,
                p,
                entropy=scratch_h,
                coverage=scratch_c,
                assumed_measurement_entropy=(
                    self.utility.weights.assumed_measurement_entropy
                ),
            )
            if upd["cells"]:
                ii, jj = upd["indices"]
                scratch_h[ii, jj] = upd["posterior"]
                scratch_c[ii, jj] = upd["posterior_coverage"]
        free = grid.passable
        entropy_after = (
            float(scratch_h[free].mean()) if free.any() else entropy_before
        )

        return PlanResult(
            poses=committed,
            evaluation=best.evaluation,
            operator=best.name,
            candidates=candidates,
            entropy_before=entropy_before,
            entropy_after=entropy_after,
            planning_time_s=time.perf_counter() - t0,
            frontier_cells=frontier_count,
        )

    # -- candidate generation ---------------------------------------------- #
    def _build_candidates(
        self,
        pose: Pose,
        reference_route: Optional[Sequence[Pose]],
        frontier: np.ndarray,
    ) -> List[PathCandidate]:
        cfg = self.config

        # Lateral deviation from the reference sweep is charged by the utility,
        # which is what keeps the published sweep as the default behaviour on a
        # field where nothing in particular is known.
        corridor_xy = self._corridor_array(reference_route)

        # (name, poses) pairs, collected before any scoring.  Candidates are
        # ranked on a *sum* over their horizon, so one that stops early banks
        # no travel cost for the leg it never flies and wins by being lazy.
        # Every candidate must therefore commit to the same number of
        # observation steps; anything shorter is not a plan, it is a truncated
        # one.
        raw: List[Tuple[str, List[Pose]]] = []

        def stage(name: str, poses: List[Pose]):
            if not poses:
                return
            if any(existing == name for existing, _ in raw):
                return
            raw.append((name, poses))

        # 0. The baseline: keep following the reference boustrophedon route.
        stage("reference_sweep", self._reference_candidate(pose, reference_route))

        # 1. Uncertainty-frontier excursions: detour to the unresolved region,
        #    then rejoin the reference sweep.  Scored as one connected path so
        #    the travel penalty charges the full round trip; a detour that does
        #    not rejoin is a coverage hole, not a plan.
        for rank, cell in enumerate(self._rank_frontier(pose, frontier)[: cfg.frontier_targets]):
            stage(
                "frontier_astar_{}".format(rank),
                self._excursion_candidate(pose, cell, reference_route),
            )

        # 2. Straight marches on an angular fan - cheap, and they capture the
        #    common case where a small heading tweak beats a detour.
        for off in cfg.fan_offsets_deg:
            stage("fan_{:+d}deg".format(int(off)), self._fan_path(pose, off))

        # 3. Coverage repair: shortest path to the most overdue cell in the
        #    coverage map.  Detours and wind push the vehicle off the reference
        #    corridor, and those holes have to be patched or the mission never
        #    reaches its coverage target.  Gated on the active configuration:
        #    the holes this patches are the ones active perception created, so
        #    a reference-only baseline must not be handed it.
        if cfg.enable_coverage_repair:
            stage("coverage_repair", self._coverage_repair_candidate(pose))

        if not raw:
            return []

        # Length the comparison is held at: the reference route's own horizon
        # when it is still ahead of the vehicle, otherwise the longest plan any
        # generator managed.  Falling back rather than demanding a fixed length
        # matters once a short route file is exhausted - the excursion and
        # repair generators are then the only things still driving the mission.
        baseline_len = next(
            (len(poses) for name, poses in raw if name == "reference_sweep"), 0
        )
        required = baseline_len or max(len(poses) for _, poses in raw)

        # Filter on length first, cap second: truncating before the filter can
        # silently discard every candidate that was actually comparable.
        selected = [(name, poses) for name, poses in raw if len(poses) == required]
        return [
            PathCandidate(
                name,
                poses,
                self.utility.evaluate_path(
                    grid=self.grid,
                    poses=poses,
                    start_pose=pose,
                    corridor_xy=corridor_xy,
                ),
            )
            for name, poses in selected[: cfg.max_candidates]
        ]

    @staticmethod
    def _corridor_array(
        reference_route: Optional[Sequence[Pose]],
    ) -> Optional[np.ndarray]:
        """Reference route as an ``(N, 2)`` array, thinned for fast distance queries."""
        if not reference_route or len(reference_route) < 2:
            return None
        pts = np.array([p.xy for p in reference_route], dtype=float)
        # A few hundred segments describe a sweep to within a cell, and keep
        # the per-candidate distance query cheap.
        if pts.shape[0] > 400:
            step = int(np.ceil(pts.shape[0] / 400.0))
            pts = pts[::step]
        return pts

    def _reference_candidate(
        self, pose: Pose, reference_route: Optional[Sequence[Pose]]
    ) -> List[Pose]:
        """Project the remaining reference route onto observation poses.

        The route is resampled by arc length rather than snapped to its own
        waypoints: ``TurnWPs.txt`` files and generated sweeps have wildly
        different waypoint densities, and observation poses must be evenly
        spaced for the travel term of the utility to be comparable across
        candidates.  Execution starts at the projection of the current pose
        onto the route, so the baseline never doubles back to pick up a leg it
        has already flown.
        """
        if not reference_route:
            return []
        return self._resample_polyline(reference_route, pose)

    def _excursion_candidate(
        self,
        pose: Pose,
        frontier_cell: Cell,
        reference_route: Optional[Sequence[Pose]],
    ) -> List[Pose]:
        """Detour to ``frontier_cell`` and rejoin the reference sweep.

        Built as a single cell-level path - entropy-biased A* out, plain A*
        back onto the nearest remaining reference waypoint - so the observation
        poses are spaced uniformly across the whole manoeuvre and the utility
        sees the true cost of the round trip.
        """
        cfg = self.config
        grid = self.grid

        start_cell = grid.nearest_free_cell(pose.x, pose.y)
        if start_cell is None:
            return []

        outbound = self._a_star_cells(start_cell, frontier_cell, cfg.entropy_bias)
        if not outbound:
            return []

        cells = list(outbound)
        if reference_route:
            remaining = [
                p for p in reference_route
                if np.linalg.norm(p.xy - pose.xy) > 1e-6
            ]
            if remaining:
                rejoin_pose = remaining[0]
                rejoin_cell = grid.nearest_free_cell(rejoin_pose.x, rejoin_pose.y)
                if rejoin_cell is not None and rejoin_cell != outbound[-1]:
                    inbound = self._a_star_cells(outbound[-1], rejoin_cell, 0.0)
                    if inbound:
                        cells.extend(inbound[1:])

        pts = [grid.cell_center(*c) for c in cells]
        return self._resample(pts, pose)

    def _coverage_repair_candidate(self, pose: Pose) -> List[Pose]:
        """Shortest path to the most overdue cell in the coverage map.

        Ranks under-covered cells by ``(1 - coverage) / distance`` so the
        search targets a real hole rather than the nearest slightly-thin patch,
        then routes to it with plain A* (this is a coverage objective, not an
        information one, so no entropy bias).
        """
        cfg = self.config
        grid = self.grid

        deficit = np.where(grid.passable, 1.0 - grid.coverage, 0.0)
        candidates = np.argwhere(deficit > cfg.repair_deficit_threshold)
        if candidates.size == 0:
            return []

        centres = np.column_stack(
            (
                grid.config.x_min + (candidates[:, 0] + 0.5) * grid.config.resolution,
                grid.config.y_min + (candidates[:, 1] + 0.5) * grid.config.resolution,
            )
        )
        dist = np.linalg.norm(centres - pose.xy, axis=1)
        keep = dist >= max(cfg.step_length, 0.5 * cfg.horizon * cfg.step_length)
        if not keep.any():
            return []
        candidates = candidates[keep]
        dist = dist[keep]

        score = deficit[candidates[:, 0], candidates[:, 1]] / dist
        best = candidates[int(np.argmax(score))]

        start_cell = grid.nearest_free_cell(pose.x, pose.y)
        if start_cell is None:
            return []
        cells = self._a_star_cells(start_cell, (int(best[0]), int(best[1])), 0.0)
        if not cells:
            return []
        return self._resample([grid.cell_center(*c) for c in cells], pose)

    def _fan_path(self, pose: Pose, offset_deg: float) -> List[Pose]:
        """A straight march at ``pose.yaw + offset``, at exact ``step_length``.

        The march is abandoned as soon as it would leave traversable ground.
        Poses are *not* snapped to cell centres: snapping would make the fan
        advance less ground per horizon than the reference sweep, and the
        utility compares candidates over a fixed horizon, so unequal advance
        would make the comparison meaningless.
        """
        cfg = self.config
        heading = math.radians(pose.yaw_deg + offset_deg)
        dx = math.cos(heading) * cfg.step_length
        dy = math.sin(heading) * cfg.step_length

        out: List[Pose] = []
        for k in range(1, cfg.horizon + 1):
            x = pose.x + dx * k
            y = pose.y + dy * k
            ci, cj = self.grid.world_to_cell(x, y)
            if not self.grid.contains(ci, cj) or not self.grid.passable[ci, cj]:
                break
            out.append(Pose(x, y, pose.yaw_deg + offset_deg))
        return out

    # -- frontier handling -------------------------------------------------- #
    def _rank_frontier(self, pose: Pose, frontier: np.ndarray) -> List[Cell]:
        """Rank frontier cells by the value of an excursion to reach them.

        Score is ``entropy * local_uncertain_density / travel_distance``:

        * ``entropy`` - how unresolved the cell itself is.
        * ``local_uncertain_density`` - how much *unresolved area surrounds*
          it, computed from an integral image.  Without this term the ranking
          collapses onto whichever uncertain cell happens to be nearest, and a
          single stray cell outranks a genuinely unobserved region.
        * ``travel_distance`` - the excursion has to be worth the detour.

        Cells closer than ``min_excursion`` are dropped: the current observation
        is about to resolve them anyway, and including them makes every
        excursion degenerate into holding station.
        """
        if frontier.size == 0:
            return []
        cfg = self.config
        grid = self.grid

        centres = np.column_stack(
            (
                grid.config.x_min + (frontier[:, 0] + 0.5) * grid.config.resolution,
                grid.config.y_min + (frontier[:, 1] + 0.5) * grid.config.resolution,
            )
        )
        dist = np.linalg.norm(centres - pose.xy, axis=1)

        # An excursion shorter than half the horizon advance is not worth
        # planning for: the reference sweep would cover that ground anyway.
        min_excursion = max(
            cfg.step_length,
            0.5 * cfg.horizon * cfg.step_length,
        )
        keep = dist >= min_excursion
        if not keep.any():
            return []
        frontier = frontier[keep]
        dist = dist[keep]

        density = self._uncertain_density(
            radius=max(2.0 * cfg.step_length, self.sensor.config.max_ground_radius)
        )
        values = grid.entropy[frontier[:, 0], frontier[:, 1]]
        local = density[frontier[:, 0], frontier[:, 1]]

        score = values * local / dist
        order = np.argsort(-score)
        return [(int(frontier[i, 0]), int(frontier[i, 1])) for i in order]

    def _uncertain_density(self, radius: float) -> np.ndarray:
        """Fraction of uncertain, traversable area within ``radius`` of each cell.

        Uses a summed-area (integral) image, so the whole density field costs
        one pass over the raster regardless of the radius.
        """
        grid = self.grid
        uncertain = (
            grid.passable
            & (grid.entropy >= self.utility.weights.frontier_entropy_threshold)
        ).astype(np.float64)

        # Integral image with a leading zero row/column so window sums are a
        # four-lookup op with no boundary special-casing.
        integral = np.zeros((grid.nx + 1, grid.ny + 1), dtype=np.float64)
        integral[1:, 1:] = np.cumsum(np.cumsum(uncertain, axis=0), axis=1)

        cell_r = max(int(round(radius / grid.config.resolution)), 1)
        ii = np.arange(grid.nx)
        jj = np.arange(grid.ny)
        i0 = np.clip(ii - cell_r, 0, grid.nx)
        i1 = np.clip(ii + cell_r + 1, 0, grid.nx)
        j0 = np.clip(jj - cell_r, 0, grid.ny)
        j1 = np.clip(jj + cell_r + 1, 0, grid.ny)

        total = (
            integral[np.ix_(i1, j1)]
            - integral[np.ix_(i0, j1)]
            - integral[np.ix_(i1, j0)]
            + integral[np.ix_(i0, j0)]
        )
        area = np.outer(i1 - i0, j1 - j0).astype(np.float64)
        return total / np.maximum(area, 1.0)

    def _cell_pose(self, cell: Cell) -> Pose:
        x, y = self.grid.cell_center(*cell)
        return Pose(x, y, 0.0)

    # -- search ------------------------------------------------------------- #
    def _a_star_path(
        self,
        start: Pose,
        goal: Pose,
        entropy_bias: float = 0.0,
    ) -> List[Pose]:
        """Entropy-biased A* between two poses, resampled to observation poses."""
        grid = self.grid
        start_cell = grid.nearest_free_cell(start.x, start.y)
        goal_cell = grid.nearest_free_cell(goal.x, goal.y)
        if start_cell is None or goal_cell is None:
            return []

        cells = self._a_star_cells(start_cell, goal_cell, entropy_bias)
        if not cells:
            return []

        pts = [grid.cell_center(*c) for c in cells]
        return self._resample(pts, start)

    def _a_star_cells(
        self, start: Cell, goal: Cell, entropy_bias: float
    ) -> List[Cell]:
        """8-connected A* with an optional uncertainty discount on cell entry."""
        grid = self.grid
        if start == goal:
            return [start]

        res = grid.config.resolution
        bias = float(np.clip(entropy_bias, 0.0, 1.0))

        def heuristic(c: Cell) -> float:
            return (
                math.hypot(c[0] - goal[0], c[1] - goal[1]) * res
            )

        open_heap: List[Tuple[float, int, Cell]] = []
        counter = 0
        heapq.heappush(open_heap, (heuristic(start), counter, start))
        came_from: Dict[Cell, Cell] = {}
        g_score: Dict[Cell, float] = {start: 0.0}
        closed = set()
        expansions = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1
            if expansions > self.config.astar_max_expansions:
                return []
            if current == goal:
                break

            for nb in grid.neighbors(current, connectivity=8):
                if nb in closed:
                    continue
                step = math.hypot(nb[0] - current[0], nb[1] - current[1]) * res
                # High uncertainty makes a cell cheap to enter: information is
                # worth travel.
                h_cell = float(grid.entropy[nb[0], nb[1]])
                step *= 1.0 - bias * h_cell
                tentative = g_score[current] + step
                if tentative < g_score.get(nb, math.inf) - 1e-12:
                    g_score[nb] = tentative
                    came_from[nb] = current
                    counter += 1
                    heapq.heappush(
                        open_heap, (tentative + heuristic(nb), counter, nb)
                    )

        if goal not in g_score:
            return []

        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    def _resample_polyline(
        self, points: Sequence[Pose], start: Pose
    ) -> List[Pose]:
        """Walk a polyline from the projection of ``start``, emitting an
        observation pose every ``step_length`` metres.

        The projection must be the nearest point on the nearest *segment*, not
        the nearest vertex.  Snapping to vertices livelocks whenever the route
        is sampled more coarsely than ``step_length``: the vehicle advances
        6 m, is still closest to the vertex it just left, and is handed the
        same target again forever.  ``TurnWPs.txt`` files are exactly that
        coarse, so this is not a corner case.

        The first emitted pose is a full ``step_length`` ahead of the vehicle,
        matching the spacing of the other candidate families so that the travel
        term in the utility is comparable across them.
        """
        cfg = self.config
        pts = np.array([p.xy for p in points], dtype=float)
        if pts.shape[0] < 2:
            return []

        a = pts[:-1]
        b = pts[1:]
        ab = b - a
        seg_len = np.linalg.norm(ab, axis=1)
        keep = seg_len > 1e-9
        a, b, ab, seg_len = a[keep], b[keep], ab[keep], seg_len[keep]
        if seg_len.size == 0:
            return []

        cum = np.concatenate(([0.0], np.cumsum(seg_len)))
        total = float(cum[-1])

        # -- project the current pose onto the polyline ------------------
        ap = start.xy - a
        t = np.clip(
            np.einsum("ij,ij->i", ap, ab) / (seg_len ** 2), 0.0, 1.0
        )
        closest = a + t[:, None] * ab
        best = int(np.argmin(np.linalg.norm(start.xy - closest, axis=1)))
        s0 = float(cum[best] + t[best] * seg_len[best])

        arclengths = s0 + cfg.step_length * np.arange(1, cfg.horizon + 1)
        arclengths = arclengths[arclengths <= total + 1e-9]
        if arclengths.size == 0:
            return []

        seg_idx = np.clip(
            np.searchsorted(cum, arclengths, side="right") - 1, 0, a.shape[0] - 1
        )
        u = (arclengths - cum[seg_idx]) / seg_len[seg_idx]
        xs = a[seg_idx, 0] + u * ab[seg_idx, 0]
        ys = a[seg_idx, 1] + u * ab[seg_idx, 1]

        out: List[Pose] = []
        prev = start
        for x, y in zip(xs, ys):
            yaw = heading_between(prev, Pose(float(x), float(y), prev.yaw_deg))
            out.append(Pose(float(x), float(y), yaw))
            prev = out[-1]
        return out

    def _resample(self, points: Sequence[Tuple[float, float]], start: Pose) -> List[Pose]:
        """Decimate a dense cell path into observation poses."""
        cfg = self.config
        if not points:
            return []
        poses = [Pose(float(p[0]), float(p[1]), 0.0) for p in points]
        out = self._resample_polyline(poses, start)
        if not out:
            last = poses[-1]
            yaw = heading_between(start, last)
            out.append(Pose(last.x, last.y, yaw))
        return out

    # -- reference route construction --------------------------------------- #
    @staticmethod
    def lawnmower_from_grid(
        grid: CoverageGrid, lane_spacing: float, lane_axis: str = "north"
    ) -> List[Pose]:
        """Obstacle- and concavity-aware lawnmower route over a belief raster.

        Sweeps the traversable mask row by row instead of clipping against the
        polygon boundary.  Contiguous free runs become lane segments, so
        obstacles, no-fly zones and non-convex field shapes are handled without
        any polygon boolean operations, and the emitted route never crosses a
        blocked cell.

        ``lane_axis="north"`` sweeps lanes running north with the cross-track
        axis to the east; ``"east"`` does the converse.
        """
        if lane_axis == "north":
            cross_size, sweep_size = grid.ny, grid.nx
        else:
            cross_size, sweep_size = grid.nx, grid.ny

        # Lane centres, alternating direction so consecutive segments connect.
        res = grid.config.resolution
        lane_count = max(int(round((cross_size * res) / max(lane_spacing, 1e-6))), 1)

        route: List[Pose] = []
        for lane_idx in range(lane_count):
            cross_pos = (lane_idx + 0.5) * (cross_size * res / lane_count)
            cross_cell = int(min(cross_pos / res, cross_size - 1))

            if lane_axis == "north":
                run_mask = grid.passable[:, cross_cell]
            else:
                run_mask = grid.passable[cross_cell, :]

            segments = RollingHorizonPlanner._free_runs(run_mask)
            if lane_idx % 2 == 1:
                segments = segments[::-1]

            for start, stop in segments:
                order = range(start, stop + 1)
                if lane_idx % 2 == 1:
                    order = range(stop, start - 1, -1)
                for sweep_cell in order:
                    if lane_axis == "north":
                        x, y = grid.cell_center(sweep_cell, cross_cell)
                    else:
                        x, y = grid.cell_center(cross_cell, sweep_cell)
                    route.append(Pose(x, y, 0.0))

        if not route:
            return []

        # Assign headings along the route direction.
        for k, pose in enumerate(route):
            if k + 1 < len(route):
                dx = route[k + 1].x - pose.x
                dy = route[k + 1].y - pose.y
            elif k > 0:
                dx = pose.x - route[k - 1].x
                dy = pose.y - route[k - 1].y
            else:
                dx, dy = 1.0, 0.0
            yaw = math.degrees(math.atan2(dy, dx)) if (dx or dy) else 0.0
            route[k] = Pose(pose.x, pose.y, yaw)

        return RollingHorizonPlanner._decimate(route, lane_spacing)

    @staticmethod
    def _free_runs(mask: np.ndarray) -> List[Tuple[int, int]]:
        """Inclusive ``(start, stop)`` index runs of ``True`` in a 1-D mask."""
        runs: List[Tuple[int, int]] = []
        start = None
        for k, free in enumerate(mask):
            if free and start is None:
                start = k
            elif not free and start is not None:
                runs.append((start, k - 1))
                start = None
        if start is not None:
            runs.append((start, len(mask) - 1))
        return runs

    @staticmethod
    def _decimate(poses: Sequence[Pose], min_spacing: float) -> List[Pose]:
        """Thin a dense route down to waypoints at least ``min_spacing`` apart,
        always keeping the final waypoint so the sweep terminates on the edge."""
        out: List[Pose] = []
        last = None
        for pose in poses:
            if last is None or math.hypot(pose.x - last.x, pose.y - last.y) >= min_spacing:
                out.append(pose)
                last = pose
        if poses and (not out or out[-1] is not poses[-1]):
            out.append(poses[-1])
        return out

    @staticmethod
    def boustrophedon_route(
        polygon_xy: Sequence[Sequence[float]],
        lane_spacing: float,
        altitude_axis: str = "north",
    ) -> List[Pose]:
        """Build a lawnmower sweep of ``polygon_xy``.

        This reproduces the geometry that ``TurnWPs.txt`` encodes in the
        upstream repository, but computes it from the polygon so that the
        planner has a route even when no pre-baked file is available.

        ``lane_spacing`` should be ``ground_width * (1 - side_lap)`` for a
        proper coverage split.
        """
        poly = np.asarray(polygon_xy, dtype=float)
        if poly.shape[0] < 3:
            return []
        if altitude_axis == "north":
            sweep_idx, cross_idx = 1, 0
        else:
            sweep_idx, cross_idx = 0, 1

        lo = float(poly[:, cross_idx].min())
        hi = float(poly[:, cross_idx].max())
        lanes = int(math.ceil((hi - lo) / max(lane_spacing, 1e-6))) + 1

        route: List[Pose] = []
        for lane in range(lanes):
            cross = lo + lane * lane_spacing
            if cross > hi:
                break
            pts = RollingHorizonPlanner._cross_section(
                poly, cross, cross_idx, sweep_idx
            )
            if not pts:
                continue
            # Alternate direction so consecutive lanes connect end-to-end.
            if lane % 2 == 1:
                pts = pts[::-1]
            for a, b in pts:
                route.append(RollingHorizonPlanner._point(a, b, cross_idx, sweep_idx))
        return route

    @staticmethod
    def _point(a: float, b: float, cross_idx: int, sweep_idx: int):
        coords = [0.0, 0.0]
        coords[cross_idx] = a
        coords[sweep_idx] = b
        return Pose(coords[0], coords[1], 0.0)

    @staticmethod
    def _cross_section(
        poly: np.ndarray, cross: float, cross_idx: int, sweep_idx: int
    ) -> List[Tuple[float, float]]:
        """Intersections of the scan line ``cross`` with the polygon edges.

        Returns ``[(cross, s0), (cross, s1)]`` for the first inside interval,
        which is the standard lawnmower lane for a simple polygon.
        """
        hits: List[float] = []
        n = poly.shape[0]
        j = n - 1
        for i in range(n):
            yi, yj = poly[i, cross_idx], poly[j, cross_idx]
            if (yi > cross) != (yj > cross):
                xi, xj = poly[i, sweep_idx], poly[j, sweep_idx]
                denom = yj - yi
                if abs(denom) < 1e-12:
                    j = i
                    continue
                t = (cross - yi) / denom
                hits.append(xi + t * (xj - xi))
            j = i
        if len(hits) < 2:
            return []
        hits.sort()
        lo, hi = hits[0], hits[-1]
        if hi - lo < 1e-6:
            return []
        return [(lo, hi)]
