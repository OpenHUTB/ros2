"""Dynamic Window Approach (DWA) core for a differential-drive robot.

This module is deliberately free of ROS imports so that the planner can be
unit-tested and simulated on a plain Python installation.  The ROS 2 node in
``main_dwa.py`` only adapts topics/parameters to the API defined here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class DWAParams:
    """Tunable parameters of the planner (all SI units)."""

    max_v: float = 0.80          # m/s
    min_v: float = -0.15         # m/s
    max_w: float = 1.80          # rad/s
    max_acc_v: float = 1.20      # m/s^2
    max_acc_w: float = 3.00      # rad/s^2
    v_res: float = 0.10          # m/s per sample
    w_res: float = 0.10          # rad/s per sample
    dt: float = 0.10             # s, control period and rollout step
    predict_time: float = 1.50   # s, rollout horizon
    robot_radius: float = 0.22   # m
    safety_margin: float = 0.08  # m, extra clearance kept by the planner
    heading_weight: float = 0.60
    goal_weight: float = 0.50
    clearance_weight: float = 0.60
    velocity_weight: float = 1.50
    goal_tolerance: float = 0.25  # m
    max_scan_points: int = 120    # downsample before distance evaluation
    clearance_norm: float = 0.70  # m, distance at which clearance cost is zero
    allow_reverse: bool = True

    def validation_errors(self) -> List[str]:
        errors: List[str] = []
        if self.max_v <= 0.0:
            errors.append("max_v must be positive")
        if self.min_v > 0.0:
            errors.append("min_v must not exceed 0")
        if self.max_w <= 0.0:
            errors.append("max_w must be positive")
        if self.dt <= 0.0:
            errors.append("dt must be positive")
        if self.predict_time < self.dt:
            errors.append("predict_time must be >= dt")
        if self.robot_radius <= 0.0:
            errors.append("robot_radius must be positive")
        if self.v_res <= 0.0 or self.w_res <= 0.0:
            errors.append("resolutions must be positive")
        return errors


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw, self.v, self.w], dtype=float)


@dataclass
class PlanResult:
    v: float
    w: float
    trajectory: np.ndarray            # (T, 2) predicted positions
    cost: float
    heading_cost: float
    goal_cost: float
    clearance_cost: float
    velocity_cost: float
    min_clearance: float              # m, distance to nearest scan point
    source: str = "dwa"               # "dwa" | "rotate" | "goal"
    feasible_samples: int = 0


def scan_points_from_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    robot_pose: RobotState,
    range_min: float = 0.05,
    range_max: float = 30.0,
    max_points: int = 120,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
) -> np.ndarray:
    """Convert polar LaserScan ranges to world-frame Cartesian points.

    Invalid (NaN/inf/out-of-range) beams are dropped.  The result is
    downsampled uniformly to at most ``max_points`` points.

    ``laser_x``/``laser_y`` are the laser origin in the robot base frame;
    when they are zero the scan is assumed to be mounted at the odom origin,
    which is the convention used by the offline harness.
    """
    ranges_arr = np.asarray(ranges, dtype=float)
    if ranges_arr.size == 0:
        return np.empty((0, 2), dtype=float)
    angles = angle_min + np.arange(ranges_arr.size, dtype=float) * angle_increment
    valid = np.isfinite(ranges_arr) & (ranges_arr >= range_min) & (ranges_arr <= range_max)
    ranges_arr = ranges_arr[valid]
    angles = angles[valid]
    if ranges_arr.size == 0:
        return np.empty((0, 2), dtype=float)
    if ranges_arr.size > max_points:
        idx = np.linspace(0, ranges_arr.size - 1, max_points).astype(int)
        ranges_arr = ranges_arr[idx]
        angles = angles[idx]
    cos_yaw = math.cos(robot_pose.yaw)
    sin_yaw = math.sin(robot_pose.yaw)
    local_x = ranges_arr * np.cos(angles) + laser_x
    local_y = ranges_arr * np.sin(angles) + laser_y
    xs = robot_pose.x + local_x * cos_yaw - local_y * sin_yaw
    ys = robot_pose.y + local_x * sin_yaw + local_y * cos_yaw
    return np.column_stack([xs, ys])


class DWA:
    """Dynamic Window Approach planner."""

    def __init__(self, params: Optional[DWAParams] = None) -> None:
        self.params = params or DWAParams()
        errors = self.params.validation_errors()
        if errors:
            raise ValueError("; ".join(errors))
        self._rollout_steps = max(1, int(round(self.params.predict_time / self.params.dt)))
        self._rollout_t = np.arange(1, self._rollout_steps + 1, dtype=float) * self.params.dt

    # ------------------------------------------------------------------
    # Dynamic window and sampling
    # ------------------------------------------------------------------
    def dynamic_window(self, state: RobotState, admissible_v: float) -> Tuple[float, float, float, float]:
        """Return (v_low, v_high, w_low, w_high) of the dynamic window."""
        p = self.params
        v_low = max(p.min_v if p.allow_reverse else 0.0, state.v - p.max_acc_v * p.dt)
        v_high = min(p.max_v, admissible_v, state.v + p.max_acc_v * p.dt)
        w_low = max(-p.max_w, state.w - p.max_acc_w * p.dt)
        w_high = min(p.max_w, state.w + p.max_acc_w * p.dt)
        return v_low, v_high, w_low, w_high

    @staticmethod
    def _sample_range(low: float, high: float, step: float) -> np.ndarray:
        if high < low:
            return np.empty(0, dtype=float)
        count = int(math.floor((high - low) / step + 1e-9)) + 1
        values = low + np.arange(count, dtype=float) * step
        if values.size == 0 or values[-1] < high - 1e-9:
            values = np.append(values, high)
        return values

    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------
    def _rollout(self, state: RobotState, v: np.ndarray, w: np.ndarray) -> np.ndarray:
        """Roll out constant (v, w) commands.  Returns (N, T, 2) positions."""
        t = self._rollout_t[None, :]
        vv = v[:, None]
        ww = w[:, None]
        yaw = state.yaw
        straight = np.abs(ww) < 1e-6
        # Avoid division by zero: use the straight-line formula where |w| ~ 0.
        safe_w = np.where(straight, 1.0, ww)
        radius = vv / safe_w
        x_arc = state.x + radius * (np.sin(yaw + ww * t) - math.sin(yaw))
        y_arc = state.y - radius * (np.cos(yaw + ww * t) - math.cos(yaw))
        x_line = state.x + vv * t * math.cos(yaw)
        y_line = state.y + vv * t * math.sin(yaw)
        xs = np.where(straight, x_line, x_arc)
        ys = np.where(straight, y_line, y_arc)
        return np.stack([xs, ys], axis=2)

    # ------------------------------------------------------------------
    # Main API
    # ------------------------------------------------------------------
    def plan(self, state: RobotState, goal: Tuple[float, float], points: np.ndarray) -> PlanResult:
        p = self.params
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        goal_dist = math.hypot(goal[0] - state.x, goal[1] - state.y)
        if goal_dist <= p.goal_tolerance:
            return PlanResult(0.0, 0.0, np.array([[state.x, state.y]]), 0.0,
                              0.0, 0.0, 0.0, 0.0, self._min_clearance(state, points),
                              source="goal")

        admissible_v = p.max_v
        v_low, v_high, w_low, w_high = self.dynamic_window(state, admissible_v)

        v_samples = self._sample_range(v_low, v_high, p.v_res)
        w_samples = self._sample_range(w_low, w_high, p.w_res)
        if v_samples.size == 0 or w_samples.size == 0:
            return self._rotate_in_place(state, goal, points)

        vv, ww = np.meshgrid(v_samples, w_samples, indexing="ij")
        v_flat = vv.ravel()
        w_flat = ww.ravel()

        trajectories = self._rollout(state, v_flat, w_flat)  # (N, T, 2)
        min_clear = self._trajectory_clearance(trajectories, points)
        needed = p.robot_radius + p.safety_margin
        feasible = min_clear >= needed

        if not np.any(feasible):
            return self._rotate_in_place(state, goal, points)

        idx = np.flatnonzero(feasible)
        traj = trajectories[idx]
        min_clear = min_clear[idx]
        v_flat = v_flat[idx]
        w_flat = w_flat[idx]

        final = traj[:, -1, :]
        final_yaw = state.yaw + w_flat * p.predict_time
        heading_err = np.abs(np.array([
            wrap_angle(math.atan2(goal[1] - y, goal[0] - x) - fy)
            for (x, y), fy in zip(final, final_yaw)
        ]))
        heading_cost = heading_err / math.pi
        final_dist = np.hypot(goal[0] - final[:, 0], goal[1] - final[:, 1])
        goal_cost = np.clip(final_dist / max(goal_dist, 1e-6), 0.0, 1.0)
        clearance_cost = np.clip((p.clearance_norm - (min_clear - p.robot_radius)) /
                                 max(p.clearance_norm, 1e-6), 0.0, 1.0)
        velocity_cost = np.clip((p.max_v - v_flat) / max(p.max_v - min(p.min_v, 0.0), 1e-6),
                                0.0, 1.0)

        total = (p.heading_weight * heading_cost +
                 p.goal_weight * goal_cost +
                 p.clearance_weight * clearance_cost +
                 p.velocity_weight * velocity_cost)
        best = int(np.argmin(total))
        best_v = float(v_flat[best])
        best_clear = float(min_clear[best])
        # Anti-creep recovery: if the optimizer's best option is to stand still
        # while an obstacle is only just outside the safety margin, rotate
        # toward free space instead of inching forward forever.
        if abs(best_v) < 0.05 and goal_dist > 0.6 and best_clear < needed + 0.15:
            return self._rotate_in_place(state, goal, points)
        return PlanResult(
            v=best_v,
            w=float(w_flat[best]),
            trajectory=traj[best],
            cost=float(total[best]),
            heading_cost=float(heading_cost[best]),
            goal_cost=float(goal_cost[best]),
            clearance_cost=float(clearance_cost[best]),
            velocity_cost=float(velocity_cost[best]),
            min_clearance=float(min_clear[best]),
            source="dwa",
            feasible_samples=int(idx.size),
        )

    def _rotate_in_place(self, state: RobotState, goal: Tuple[float, float],
                         points: np.ndarray) -> PlanResult:
        """Fallback when every forward sample is blocked: turn toward free space.

        The robot compares the free distance on its left and right forward
        quadrants and rotates toward the roomier side.  Turning blindly toward
        the goal (the naive fallback) makes the robot stare at the obstacle and
        stay trapped, which is exactly the failure mode this rule avoids.
        """
        p = self.params
        trajectory = np.array([[state.x, state.y]])
        best_w = p.max_w * 0.6
        if points.size:
            rel = points - np.array([state.x, state.y])
            dist = np.hypot(rel[:, 0], rel[:, 1])
            forward = (rel[:, 0] * math.cos(state.yaw) + rel[:, 1] * math.sin(state.yaw)) > 0.0
            cross = (math.cos(state.yaw) * rel[:, 1] - math.sin(state.yaw) * rel[:, 0])
            left = forward & (cross > 0.0)
            right = forward & (cross < 0.0)
            left_score = float(dist[left].min()) if np.any(left) else float("inf")
            right_score = float(dist[right].min()) if np.any(right) else float("inf")
            best_w = p.max_w * 0.6 if left_score > right_score else -p.max_w * 0.6
        # Respect the same angular acceleration limit as the DWA window; without
        # this the fallback would command a step change on a real robot.
        w_step = p.max_acc_w * p.dt
        best_w = clamp(best_w, state.w - w_step, state.w + w_step)
        best_w = clamp(best_w, -p.max_w, p.max_w)
        return PlanResult(0.0, best_w, trajectory, 1e6, 1.0, 1.0, 1.0, 1.0,
                          self._min_clearance(state, points), source="rotate")

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    def _min_clearance(self, state: RobotState, points: np.ndarray) -> float:
        if points.size == 0:
            return float("inf")
        return float(np.hypot(points[:, 0] - state.x, points[:, 1] - state.y).min())

    @staticmethod
    def _trajectory_clearance(trajectories: np.ndarray, points: np.ndarray) -> np.ndarray:
        """Minimum distance from every rollout to the scan points.

        ``trajectories`` has shape (N, T, 2), ``points`` (M, 2); the returned
        array has shape (N,).
        """
        if points.size == 0:
            return np.full(trajectories.shape[0], np.inf)
        dx = trajectories[:, :, None, 0] - points[None, None, :, 0]
        dy = trajectories[:, :, None, 1] - points[None, None, :, 1]
        dist = np.sqrt(dx * dx + dy * dy)
        return dist.min(axis=2).min(axis=1)

