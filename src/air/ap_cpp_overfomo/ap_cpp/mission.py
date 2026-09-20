"""Mission driver that binds the belief grid, planner, sensor and controller.

The driver is intentionally free of any simulator dependency.  It advances a
kinematic point model at the commanded speed, calls the perception back-end at
each observation pose, fuses the result into the belief and asks the planner
for a fresh horizon.  The same class therefore backs:

* the offline demo (``demos/run_ap_cpp_demo.py``),
* the CI smoke test (``tests/test_ap_cpp.py``),
* and, with an AirSim pose source swapped in, the real flight script.

The loop terminates on any of: mission complete, step budget, time budget, or
the planner returning an empty horizon (nothing left worth visiting).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Optional, Sequence

import numpy as np

from ap_cpp.control import APCPPSpeedController, SpeedCommand
from ap_cpp.grid_model import CoverageGrid
from ap_cpp.planner import PlannerConfig, PlanResult, RollingHorizonPlanner, StepDiagnostics
from ap_cpp.pose import Pose, heading_between
from ap_cpp.runtime import DemoPerceptionModel, PerceptionModel
from ap_cpp.sensor import SensorModel

__all__ = ["MissionConfig", "MissionRecord", "APCPPMission"]


@dataclass
class MissionConfig:
    """Termination criteria and simulation bookkeeping."""

    coverage_target: float = 0.85
    """Fraction of the operational area that must reach full coverage."""

    entropy_target: float = 0.25
    """Mean normalised belief entropy below which the map is 'resolved'."""

    max_steps: int = 400
    """Hard cap on observation steps."""

    max_time: float = 1800.0
    """Hard cap on mission duration, in seconds."""

    stale_step_limit: int = 25
    """Abort after this many consecutive steps with negligible information gain."""

    min_useful_gain: float = 1e-4
    """Gain below which a step counts as 'stale'."""

    replan_on_arrival: bool = True
    """Re-plan whenever the committed horizon is exhausted."""

    def as_dict(self) -> dict:
        return dict(self.__dict__)


@dataclass
class MissionRecord:
    """Full mission log - the artefact the demo and the plots consume."""

    steps: List[StepDiagnostics] = field(default_factory=list)
    plans: List[dict] = field(default_factory=list)
    speed_commands: List[dict] = field(default_factory=list)
    total_distance: float = 0.0
    duration: float = 0.0
    termination: str = "unknown"

    @property
    def information_gain(self) -> float:
        return float(sum(s.information_gain for s in self.steps))

    def final_summary(self, grid: CoverageGrid) -> dict:
        return {
            "steps": len(self.steps),
            "replans": len(self.plans),
            "total_distance_m": self.total_distance,
            "duration_s": self.duration,
            "termination": self.termination,
            "cumulative_information_gain": self.information_gain,
            "mean_speed": (
                self.total_distance / self.duration if self.duration > 1e-9 else 0.0
            ),
            **grid.summary(),
        }

    def operator_histogram(self) -> Dict[str, int]:
        hist: Dict[str, int] = {}
        for s in self.steps:
            hist[s.operator] = hist.get(s.operator, 0) + 1
        return hist

    def as_dict(self) -> dict:
        return {
            "steps": [s.as_dict() for s in self.steps],
            "plans": self.plans,
            "speed_commands": self.speed_commands,
            "total_distance": self.total_distance,
            "duration": self.duration,
            "termination": self.termination,
        }


class APCPPMission:
    """Receding-horizon active perception coverage mission."""

    def __init__(
        self,
        grid: CoverageGrid,
        planner: RollingHorizonPlanner,
        perception: PerceptionModel,
        controller: Optional[APCPPSpeedController] = None,
        reference_route: Optional[Sequence[Pose]] = None,
        config: Optional[MissionConfig] = None,
        sensor: Optional[SensorModel] = None,
    ):
        self.grid = grid
        self.planner = planner
        self.perception = perception
        self.sensor = sensor or planner.sensor
        self.controller = controller or APCPPSpeedController()
        self.reference_route = list(reference_route) if reference_route else []
        self.config = config or MissionConfig()

        self.pose: Optional[Pose] = None
        self.time: float = 0.0
        self.record = MissionRecord()
        self._horizon: List[Pose] = []
        self._last_reading_entropy: float = 1.0
        self._last_coverage_ratio: float = 0.0
        self._last_planned_gain: float = 0.0
        self._stale = 0

    # -- public API -------------------------------------------------------- #
    def run(self, start_pose: Pose) -> MissionRecord:
        """Execute the mission to completion and return the log."""
        for _ in self.step(start_pose):
            pass
        return self.record

    def step(self, start_pose: Pose) -> Iterator[StepDiagnostics]:
        """Generator form of :meth:`run` - yields after every observation step.

        Useful for live plotting: the caller can render the belief grid between
        yields without the mission giving up control of its own loop.
        """
        cfg = self.config
        self.pose = start_pose
        self.time = 0.0
        self.record = MissionRecord()

        self._replan(force=True)

        while True:
            if len(self.record.steps) >= cfg.max_steps:
                self.record.termination = "max_steps"
                break
            if self.time >= cfg.max_time:
                self.record.termination = "max_time"
                break
            if self.grid.mission_complete(cfg.coverage_target, cfg.entropy_target):
                self.record.termination = "mission_complete"
                break
            if self._stale >= cfg.stale_step_limit:
                self.record.termination = "converged_no_new_information"
                break
            if not self._horizon:
                self._replan(force=True)
                if not self._horizon:
                    self.record.termination = "nothing_left_to_explore"
                    break

            target = self._horizon.pop(0)
            diag = self._execute_step(target)
            if diag is not None:
                yield diag

        self.record.duration = self.time

    def replan(self) -> PlanResult:
        """Force a replan from the current pose (exposed for the AirSim driver)."""
        return self._replan(force=True)

    def consume_pose(self) -> Optional[Pose]:
        """Pop the next committed observation pose, or ``None`` when empty.

        The AirSim driver polls this instead of :meth:`step`: it flies to the
        returned pose, then calls :meth:`observe_at` with the pose it actually
        reached - which is where model error enters the loop and gets corrected
        by the next replan.
        """
        if not self._horizon:
            self._replan(force=True)
        if not self._horizon:
            return None
        return self._horizon.pop(0)

    def observe_at(self, pose: Pose) -> StepDiagnostics:
        """Fuse a real observation at ``pose`` and log the step."""
        return self._execute_step(pose, enforce_transit=False)

    # -- internals --------------------------------------------------------- #
    def _replan(self, force: bool = False) -> PlanResult:
        if self.pose is None:
            raise RuntimeError("mission has no pose yet")
        if not force and not self.planner.should_replan(self.pose, self.time):
            return PlanResult(
                poses=[], evaluation=self.planner.utility.evaluate_path(self.grid, []),
                operator="skipped",
            )

        result = self.planner.plan(
            self.pose, reference_route=self.reference_route, now=self.time
        )
        self._horizon = list(result.poses)
        self._last_planned_gain = (
            result.evaluation.terms[0].information_gain
            if result.evaluation.terms
            else 0.0
        )

        self.record.plans.append(
            {
                "t": self.time,
                "pose": self.pose.as_tuple(),
                "operator": result.operator,
                **result.as_dict(),
            }
        )
        return result

    def _execute_step(
        self, target: Pose, enforce_transit: bool = True
    ) -> Optional[StepDiagnostics]:
        cfg = self.config
        start_xy = self.pose.xy

        travelled = float(np.linalg.norm(target.xy - start_xy))
        if enforce_transit:
            command: SpeedCommand = self.controller.command(
                coverage_ratio=self._last_coverage_ratio,
                measurement_entropy=self._last_reading_entropy,
                planned_gain=self._last_planned_gain,
                mean_entropy=self.grid.mean_entropy(),
            )
            dt = travelled / max(command.speed, 1e-6)
            self.time += dt
        else:
            command = self.controller.command(
                coverage_ratio=self._last_coverage_ratio,
                measurement_entropy=self._last_reading_entropy,
                planned_gain=self._last_planned_gain,
                mean_entropy=self.grid.mean_entropy(),
            )
        self.record.total_distance += travelled

        yaw = heading_between(self.pose, target)
        self.pose = Pose(target.x, target.y, yaw)

        entropy_before = self.grid.mean_entropy()
        reading = self.perception.perceive(self.grid, self.pose, self.time)
        gain = self.sensor.observe(
            self.grid,
            self.pose,
            measurement_entropy=reading.measurement_entropy,
            timestep=self.time,
        )
        entropy_after = self.grid.mean_entropy()

        self._last_reading_entropy = reading.measurement_entropy
        self._last_coverage_ratio = reading.coverage_ratio

        if gain < cfg.min_useful_gain:
            self._stale += 1
        else:
            self._stale = 0

        diag = StepDiagnostics(
            index=len(self.record.steps),
            operator=self.record.plans[-1]["operator"] if self.record.plans else "unknown",
            pose=self.pose,
            entropy_before=entropy_before,
            entropy_after=entropy_after,
            coverage_fraction=self.grid.coverage_fraction(),
            information_gain=gain,
            speed=command.speed,
            travelled=travelled,
            candidate_scores={
                c["name"]: c["score"] for c in self.record.plans[-1].get("candidates", [])
            }
            if self.record.plans
            else {},
            planned_pose_count=len(self._horizon) + 1,
        )
        self.record.steps.append(diag)
        self.record.speed_commands.append({"t": self.time, **command.as_dict()})
        return diag
