"""Walking control for the humanoid: pure pursuit, balance-aware speed limits,
obstacle braking, fall recovery and the mission-level controller.

The robot pose is ``(x, y, 0, yaw)`` and the simulator command is
``(walk_speed [m/s], turn_rate [rad/s])`` (positive turn rate = turn left).
:class:`MissionController` returns exactly that pair, so
``sim.step(mission.update(...))`` is the intended usage.

The lateral balance state is the safety-critical variable: the simulator
integrates ``b_dot = BALANCE_GAIN * v * omega - BALANCE_RESTORE * b`` and
registers a fall once ``|b| > BALANCE_LIMIT`` for ``FALL_HOLD`` seconds.  The
controller therefore

* limits the combined command ``|v * omega|`` (see
  :data:`planning.BALANCE_PRODUCT_LIMIT`),
* slows both walk speed and turn rate when ``|balance|`` exceeds 60 % of the
  limit,
* commands a full stop (turn in place / stand still) while ``|balance|`` is
  over the limit or right after a fall was registered, and
* brakes for frontal lidar obstacles (slow below 0.55 m, stop below 0.35 m).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .planning import (BALANCE_PRODUCT_LIMIT, DWAPlanner, GeometricPlanner,
                       GridPlanner, scene_planning_grid)

__all__ = ["PID", "PurePursuit", "SafetyFilter", "WalkingController",
           "MissionController"]

_RANGE_MAX = float(C.LIDAR_RANGE_MAX)
_RANGE_MIN = float(C.LIDAR_RANGE_MIN)

# frontal obstacle thresholds from the task specification
_HARD_CLEARANCE = 0.35       # m, stop walking
_SOFT_CLEARANCE = 0.55       # m, start slowing down
_BALANCE_SOFT = 0.60         # fraction of BALANCE_LIMIT where we slow down
_PRODUCT_MARGIN = 0.90       # keep |v*omega| below this fraction of the limit


class PID:
    """Discrete PID controller with output and integral clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 limit: Optional[float] = None,
                 i_limit: Optional[float] = None) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = None if limit is None else abs(float(limit))
        self.i_limit = None if i_limit is None else abs(float(i_limit))
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error: Optional[float] = None

    def update(self, error: float, dt: float) -> float:
        """Return the controller output for one step of length ``dt`` seconds."""
        error = float(error)
        dt = max(float(dt), 1e-6)
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._integral += error * dt
        if self.i_limit is not None:
            self._integral = max(-self.i_limit, min(self.i_limit, self._integral))
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        if self.limit is not None:
            out = max(-self.limit, min(self.limit, out))
        return float(out)


class PurePursuit:
    """Lookahead path tracker for the walking humanoid (default lookahead 0.8 m).

    ``compute`` returns ``(v_target, omega_target)`` in the simulator units.
    When the heading error is large the robot turns in place (``v = 0``), which
    both follows the path faster and keeps ``|v * omega|`` small.
    """

    def __init__(self, lookahead: float = 0.8,
                 cruise_speed: float = 0.95,
                 max_turn: float = C.MAX_TURN_RATE,
                 goal_tolerance: float = C.GOAL_TOLERANCE) -> None:
        self.lookahead = float(lookahead)
        self.cruise_speed = float(min(cruise_speed, C.MAX_SPEED))
        self.max_turn = float(min(max_turn, C.MAX_TURN_RATE))
        self.goal_tolerance = float(goal_tolerance)
        self.turn_gain = 2.2
        self.in_place_angle = 0.60      # rad, above this we turn in place
        self.last_target: Tuple[float, float] = (0.0, 0.0)
        self.last_cross_error = 0.0

    # --------------------------------------------------------------- helpers
    def _lookahead_target(self, pose: np.ndarray,
                          pts: np.ndarray) -> Tuple[float, float, float]:
        """Return ``(target_x, target_y, distance_to_path_end)``."""
        x, y = float(pose[0]), float(pose[1])
        ex, ey = float(pts[-1, 0]), float(pts[-1, 1])
        d_end = math.hypot(ex - x, ey - y)
        if pts.shape[0] == 1:
            return ex, ey, d_end
        best = (math.inf, 0, 0.0)
        for j in range(pts.shape[0] - 1):
            ax, ay = pts[j]
            bx, by = pts[j + 1]
            abx, aby = bx - ax, by - ay
            seg2 = abx * abx + aby * aby
            t = 0.0 if seg2 <= 1e-12 else \
                max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / seg2))
            px, py = ax + t * abx, ay + t * aby
            d = math.hypot(px - x, py - y)
            if d < best[0]:
                best = (d, j, t)
        _, j0, t0 = best
        ax, ay = pts[j0]
        bx, by = pts[j0 + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len <= 1e-9:
            return ex, ey, d_end
        remain = (1.0 - t0) * seg_len
        if remain >= self.lookahead:
            ratio = self.lookahead / seg_len
            return (ax + (t0 + ratio) * (bx - ax),
                    ay + (t0 + ratio) * (by - ay), d_end)
        for j in range(j0 + 1, pts.shape[0] - 1):
            sx, sy = pts[j]
            nx, ny = pts[j + 1]
            seg_len = math.hypot(nx - sx, ny - sy)
            if remain + seg_len >= self.lookahead:
                ratio = (self.lookahead - remain) / max(seg_len, 1e-9)
                return (sx + ratio * (nx - sx), sy + ratio * (ny - sy), d_end)
            remain += seg_len
        return ex, ey, d_end

    # ---------------------------------------------------------------- public
    def compute(self, pose: Sequence[float], path: Sequence[Sequence[float]],
                speed: float = 0.0,
                speed_limit: Optional[float] = None
                ) -> Tuple[float, float]:
        """Return ``(v_target, omega_target)`` for the current path."""
        self.last_target = (0.0, 0.0)
        if path is None or len(path) < 1:
            return 0.0, 0.0
        pose = np.asarray(pose, dtype=np.float64).ravel()
        pts = np.asarray(path, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0:
            return 0.0, 0.0
        x, y = float(pose[0]), float(pose[1])
        if pose.size >= 4:
            yaw = float(pose[3])
        elif pose.size >= 3:
            yaw = float(pose[2])
        else:
            yaw = 0.0
        tx, ty, d_end = self._lookahead_target(pose, pts)
        self.last_target = (tx, ty)
        self.last_cross_error = -math.sin(yaw) * (tx - x) + math.cos(yaw) * (ty - y)
        alpha = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        omega = float(np.clip(self.turn_gain * alpha,
                              -self.max_turn, self.max_turn))
        cruise = self.cruise_speed
        if speed_limit is not None:
            cruise = min(cruise, max(0.0, float(speed_limit)))
        if abs(alpha) > self.in_place_angle:
            v = 0.0
        else:
            align = max(0.0, math.cos(alpha))
            v = cruise * (0.30 + 0.70 * align)
        # endgame: walk the last metre slowly and come to a halt at the path end
        if d_end < self.goal_tolerance:
            v = 0.0
        elif d_end < 1.0:
            v *= 0.35 + 0.65 * (d_end / 1.0)
        return float(v), omega


class SafetyFilter:
    """Frontal lidar safety filter for the walking robot.

    ``clearance`` is the minimum lidar range inside a +-35 deg sector; below
    0.35 m the robot stops walking (it may still turn) and between 0.35 and
    0.55 m the walk speed is scaled down linearly.
    """

    def __init__(self, hard_clearance: float = _HARD_CLEARANCE,
                 soft_clearance: float = _SOFT_CLEARANCE,
                 half_fov_deg: float = 35.0) -> None:
        self.hard_clearance = float(hard_clearance)
        self.soft_clearance = float(soft_clearance)
        self.half_fov = math.radians(float(half_fov_deg))

    def frontal_clearance(self, scan: Sequence[float],
                          beam_angles: Sequence[float],
                          center_angle: float = 0.0) -> float:
        ranges = np.asarray(scan, dtype=np.float64).ravel()
        beams = np.asarray(beam_angles, dtype=np.float64).ravel()
        n = min(ranges.size, beams.size)
        if n == 0:
            return _RANGE_MAX
        rel = np.abs(np.asarray(
            [wrap_angle(float(b) - float(center_angle)) for b in beams[:n]],
            dtype=np.float64))
        front = rel <= self.half_fov
        valid = (np.isfinite(ranges[:n]) & (ranges[:n] < _RANGE_MAX * 0.995)
                 & (ranges[:n] > _RANGE_MIN + 0.02))
        sel = front & valid
        if not np.any(sel):
            return _RANGE_MAX
        return float(np.min(ranges[:n][sel]))

    def speed_scale(self, scan: Sequence[float], beam_angles: Sequence[float],
                    center_angle: float = 0.0) -> float:
        clearance = self.frontal_clearance(scan, beam_angles, center_angle)
        if clearance <= self.hard_clearance:
            return 0.0
        if clearance >= self.soft_clearance:
            return 1.0
        span = max(self.soft_clearance - self.hard_clearance, 1e-6)
        return float((clearance - self.hard_clearance) / span)

    def apply(self, cmd: Sequence[float], scan: Sequence[float],
              beam_angles: Sequence[float]) -> np.ndarray:
        """Scale the walk speed of a ``(v, omega)`` command for the clearance."""
        out = np.array(cmd, dtype=np.float64, copy=True)
        out[0] = float(out[0]) * self.speed_scale(scan, beam_angles)
        return out


class WalkingController:
    """Pure pursuit + balance-aware speed limits + obstacle braking.

    Combines :class:`PurePursuit` and :class:`SafetyFilter` and then enforces
    the lateral-balance safety envelope on the resulting ``(v, omega)``.
    """

    def __init__(self, lookahead: float = 0.8,
                 cruise_speed: float = 0.95,
                 max_turn: float = C.MAX_TURN_RATE) -> None:
        self.pursuit = PurePursuit(lookahead=lookahead,
                                   cruise_speed=cruise_speed,
                                   max_turn=max_turn)
        self.safety = SafetyFilter()
        self.balance_limit = float(C.BALANCE_LIMIT)
        self.product_limit = float(BALANCE_PRODUCT_LIMIT) * _PRODUCT_MARGIN
        self.last_clearance = _RANGE_MAX
        self.last_balance = 0.0
        self.recovering = False

    def clamp_balance(self, v: float, omega: float,
                      balance: float) -> Tuple[float, float]:
        """Limit a ``(v, omega)`` pair for the current lateral CoM offset."""
        b = abs(float(balance))
        soft = _BALANCE_SOFT * self.balance_limit
        if b >= self.balance_limit:
            # tipping: stand still until the CoM is back inside the support
            return 0.0, 0.0
        if b > soft:
            scale = max(0.0, (self.balance_limit - b)
                        / max(self.balance_limit - soft, 1e-6))
            scale = 0.15 + 0.85 * scale
            v *= scale
            omega *= scale
        product = abs(v * omega)
        if product > self.product_limit:
            # reduce the turn first (walking straight is the safest gait)
            if abs(omega) > 1e-6:
                omega = math.copysign(self.product_limit / max(abs(v), 1e-6),
                                      omega)
            product = abs(v * omega)
            if product > self.product_limit:
                v = math.copysign(self.product_limit / max(abs(omega), 1e-6), v)
        return float(v), float(omega)

    def compute(self, pose: Sequence[float], path: Sequence[Sequence[float]],
                scan: Sequence[float], beam_angles: Sequence[float],
                balance: float = 0.0, speed: float = 0.0,
                speed_limit: Optional[float] = None) -> Tuple[float, float]:
        """Return the balance-safe ``(v, omega)`` walking command."""
        self.last_balance = float(balance)
        if path is None or len(path) == 0:
            return 0.0, 0.0
        v, omega = self.pursuit.compute(pose, path, speed=speed,
                                        speed_limit=speed_limit)
        clearance = self.safety.frontal_clearance(scan, beam_angles)
        self.last_clearance = clearance
        if clearance <= self.safety.hard_clearance:
            v = 0.0
        elif clearance < self.safety.soft_clearance:
            span = max(self.safety.soft_clearance
                       - self.safety.hard_clearance, 1e-6)
            v *= (clearance - self.safety.hard_clearance) / span
        v, omega = self.clamp_balance(v, omega, balance)
        return v, omega


def _densify_path(path: Sequence[Sequence[float]],
                  step: float = 0.25) -> List[Tuple[float, float]]:
    """Resample a (simplified) polyline at roughly ``step`` metres."""
    if path is None or len(path) == 0:
        return []
    out: List[Tuple[float, float]] = [(float(path[0][0]), float(path[0][1]))]
    for a, b in zip(path[:-1], path[1:]):
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        dist = math.hypot(bx - ax, by - ay)
        n = max(1, int(math.ceil(dist / max(step, 1e-3))))
        for k in range(1, n + 1):
            t = k / float(n)
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


class MissionController:
    """Mission-level walking controller.

    Global A* on the indoor occupancy grid (walls, crates, pillars blocked) plus
    :class:`WalkingController` path tracking, balance-aware limits, obstacle
    braking and fall recovery.

    Parameters
    ----------
    scene_or_map:
        :class:`scenes.Scene` (its floor-plan occupancy is used) or a 2-D
        boolean blocked grid.
    resolution:
        Planning grid resolution in metres, default
        :data:`config.PLAN_RESOLUTION`.
    replan_interval:
        Maximum time (s) between re-plans; the path is also re-planned when it
        becomes invalid, the goal moves or the robot gets stuck.
    cruise_speed:
        Desired free-flow walking speed (m/s); corners, balance and obstacles
        reduce it locally.
    lookahead:
        Pure-pursuit lookahead distance (m), default 0.8.
    """

    def __init__(self, scene_or_map, resolution: float = C.PLAN_RESOLUTION,
                 replan_interval: float = 6.0,
                 cruise_speed: float = 0.95,
                 lookahead: float = 0.8) -> None:
        scene = None
        if hasattr(scene_or_map, "occupancy") or hasattr(scene_or_map, "coarse_map"):
            scene = scene_or_map
            base = scene_planning_grid(scene, resolution)
        else:
            base = np.asarray(scene_or_map, dtype=bool)
        self.scene = scene
        self.resolution = float(resolution)
        self.cruise_speed = float(min(cruise_speed, C.MAX_SPEED))
        self._base_grid = base
        # multi-level inflation: prefer a wider stand-off than the bare config
        # value so a walking robot does not try to squeeze through gaps that are
        # only just wider than its footprint; fall back to the config radius only
        # when a scene is too narrow for the wide one.
        self._radii = (C.INFLATION_RADIUS + 0.25,
                       C.INFLATION_RADIUS + 0.10,
                       max(C.ROBOT_RADIUS, C.INFLATION_RADIUS))
        self._planners: List[GridPlanner] = []
        self.planner = self._get_planner(0)
        self.walking = WalkingController(lookahead=lookahead,
                                         cruise_speed=self.cruise_speed)
        self.fallback = GeometricPlanner(scene=scene)
        self.replan_interval = float(replan_interval)
        self.path: List[Tuple[float, float]] = []
        self.replans = 0
        self.stopped_for_obstacle = 0
        self.stopped_for_cart = 0
        self.recoveries = 0
        self.target_speed = 0.0
        self.target_turn = 0.0
        self._v_profile: List[float] = []
        self._goal: Optional[Tuple[float, float]] = None
        self._timer = 0.0
        self._stuck_time = 0.0
        self._time = 0.0
        self._speed_est = 0.0
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._falls_seen = 0
        self._recovery_until = -1.0
        self._obstacle_active = False
        self._cart_active = False

    # ---------------------------------------------------------------- helpers
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        """Reset the per-episode state (path, counters and internal clock).

        Closed-loop harnesses call ``controller.reset(...)`` before every
        episode; the internal clock drives the fall-recovery window, so it has
        to restart with the episode as well.
        """
        self.path = []
        self._v_profile = []
        self.replans = 0
        self.stopped_for_obstacle = 0
        self.stopped_for_cart = 0
        self.recoveries = 0
        self.target_speed = 0.0
        self.target_turn = 0.0
        self._goal = None
        self._timer = 0.0
        self._stuck_time = 0.0
        self._time = 0.0
        self._speed_est = 0.0
        self._falls_seen = 0
        self._recovery_until = -1.0
        self._obstacle_active = False
        self._cart_active = False
        self.walking = WalkingController(
            lookahead=self.walking.pursuit.lookahead,
            cruise_speed=self.cruise_speed)
        if pose is None:
            self._prev_xy = None
        else:
            pose = np.asarray(pose, dtype=np.float64).ravel()
            self._prev_xy = (float(pose[0]), float(pose[1]))

    def _get_planner(self, index: int) -> GridPlanner:
        while len(self._planners) <= index:
            radius = self._radii[min(index, len(self._radii) - 1)]
            self._planners.append(GridPlanner(
                self._base_grid, self.resolution, radius / self.resolution))
        return self._planners[index]

    def _build_profile(self) -> None:
        """Per-waypoint speed ceiling from path curvature and clearance."""
        pts = self.path
        n = len(pts)
        prof: List[float] = []
        for i in range(n):
            v = self.cruise_speed
            if n >= 2:
                c = self.planner.clearance(pts[i][0], pts[i][1])
                if c < C.ROBOT_RADIUS + 0.30:
                    v = min(v, 0.45)
                elif c < C.ROBOT_RADIUS + 0.60:
                    v = min(v, 0.70)
            if 0 < i < n - 1:
                a1 = math.atan2(pts[i][1] - pts[i - 1][1],
                                pts[i][0] - pts[i - 1][0])
                a2 = math.atan2(pts[i + 1][1] - pts[i][1],
                                pts[i + 1][0] - pts[i][0])
                turn = abs(wrap_angle(a2 - a1))
                if turn > 0.45:
                    v = min(v, 0.45)
                elif turn > 0.22:
                    v = min(v, 0.70)
            prof.append(float(v))
        self._v_profile = prof

    def _smooth_path(self, pts: Sequence[Sequence[float]],
                     iterations: int = 25,
                     weight: float = 0.5) -> List[Tuple[float, float]]:
        """Round the corners of a dense path while keeping it in free space."""
        if len(pts) <= 2:
            return [(float(p[0]), float(p[1])) for p in pts]
        arr = np.asarray(pts, dtype=np.float64).copy()
        res = self.planner.resolution
        grid = self.planner.grid
        h, w = grid.shape
        for _ in range(int(iterations)):
            cand = (1.0 - weight) * arr + weight * 0.5 * (
                np.vstack([arr[0], arr[:-1]]) + np.vstack([arr[1:], arr[-1]]))
            cand[0] = arr[0]
            cand[-1] = arr[-1]
            cols = np.clip(np.floor(cand[:, 0] / res).astype(np.int64), 0, w - 1)
            rows = np.clip(np.floor(cand[:, 1] / res).astype(np.int64), 0, h - 1)
            free = ~grid[rows, cols]
            arr = np.where(free[:, None], cand, arr)
        return [(float(p[0]), float(p[1])) for p in arr]

    def _replan(self, pose: Sequence[float],
                goal_xy: Tuple[float, float]) -> None:
        pose = np.asarray(pose, dtype=np.float64).ravel()
        path = None
        for index in range(len(self._radii)):
            planner = self._get_planner(index)
            path = planner.plan((float(pose[0]), float(pose[1])), goal_xy)
            if path:
                self.planner = planner
                break
        if path:
            dense = _densify_path(path)
            self.path = self._smooth_path(dense)
            self._build_profile()
        else:
            self.path = []
            self._v_profile = []
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._timer = self.replan_interval
        self._stuck_time = 0.0
        self.replans += 1

    def _needs_replan(self, pose_xy: Tuple[float, float],
                      goal_xy: Tuple[float, float]) -> bool:
        if self._goal is None or not self.path:
            return True
        if math.hypot(goal_xy[0] - self._goal[0],
                      goal_xy[1] - self._goal[1]) > 0.35:
            return True
        if math.hypot(goal_xy[0] - pose_xy[0],
                      goal_xy[1] - pose_xy[1]) < 2.0:
            return False
        if self._stuck_time > 2.0:
            return True
        if self._timer <= 0.0:
            return True
        n = len(self.path)
        for k in range(0, n, 3):
            x, y = self.path[k]
            if not self.planner.is_free(x, y):
                return True
        return False

    def _cart_clearance(self, pose: Sequence[float]) -> float:
        """Distance from the robot to the (moving) cart sweep, in metres.

        The warehouse scene has one cart that shuttles slowly across the hall.
        The planning map is static, so the mission controller also looks at the
        scene's cart schedule (current position and two short look-ahead
        instants) and simply waits when the robot is about to step into the
        cart's swept corridor.  ``inf`` when the scene has no carts.
        """
        scene = self.scene
        carts = getattr(scene, "carts", None) if scene is not None else None
        if not carts:
            return float("inf")
        x, y = float(pose[0]), float(pose[1])
        best = float("inf")
        for ahead in (0.0, 0.6, 1.2):
            for cx, cy, hw, hh in scene.cart_positions(self._time + ahead):
                dx = max(abs(x - cx) - hw, 0.0)
                dy = max(abs(y - cy) - hh, 0.0)
                best = min(best, math.hypot(dx, dy))
        return best

    def _closest_index(self, x: float, y: float) -> int:
        best = (math.inf, 0)
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best[0]:
                best = (d, i)
        return best[1]

    def _profile_limit(self, index: int, lookahead: float) -> float:
        """Lowest speed ceiling on the path within ``lookahead`` metres."""
        if not self.path:
            return self.cruise_speed
        limit = self.cruise_speed
        n = len(self.path)
        i = max(0, int(index))
        remain = float(lookahead)
        while True:
            if self._v_profile:
                limit = min(limit, self._v_profile[i])
            if i >= n - 1 or remain <= 0.0:
                break
            seg = math.hypot(self.path[i + 1][0] - self.path[i][0],
                             self.path[i + 1][1] - self.path[i][1])
            remain -= seg
            i += 1
        return limit

    # ----------------------------------------------------------------- update
    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan: Sequence[float], beam_angles: Sequence[float],
               dt: float, balance: float = 0.0,
               falls: int = 0) -> np.ndarray:
        """Compute one ``(walk_speed, turn_rate)`` command for ``HumanoidSim.step``.

        ``goal`` is ``[x, y]`` or ``[x, y, 0]``; ``balance`` is the lateral CoM
        offset (positive = left) and ``falls`` the simulator fall counter.
        """
        pose = np.asarray(pose, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        dt = max(float(dt), 1e-6)
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        gx = float(goal[0])
        gy = float(goal[1])

        # ---- bookkeeping: speed estimate, clock, fall detection
        if self._prev_xy is None:
            self._speed_est = 0.0
        else:
            meas = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1]) / dt
            meas = min(meas, C.MAX_SPEED + 1.0)
            self._speed_est = 0.7 * self._speed_est + 0.3 * meas
        self._prev_xy = (x, y)
        self._time += dt
        self._timer -= dt
        if int(falls) > self._falls_seen:
            self._falls_seen = int(falls)
            self.recoveries += 1
            self._recovery_until = self._time + 0.8
        if self._time < self._recovery_until:
            self.target_speed = 0.0
            self.target_turn = 0.0
            return np.array([0.0, 0.0], dtype=np.float64)

        dist_goal = math.hypot(gx - x, gy - y)
        if self._needs_replan((x, y), (gx, gy)):
            self._replan(pose, (gx, gy))

        # ---- tipping: stop and recover in place
        if abs(float(balance)) >= C.BALANCE_LIMIT:
            self.target_speed = 0.0
            self.target_turn = 0.0
            return np.array([0.0, 0.0], dtype=np.float64)

        if dist_goal <= C.GOAL_TOLERANCE:
            self.target_speed = 0.0
            self.target_turn = 0.0
            return np.array([0.0, 0.0], dtype=np.float64)

        # ---- path tracking
        if self.path:
            index = self._closest_index(x, y)
            speed_limit = self._profile_limit(index, 1.5)
            v, omega = self.walking.compute(
                pose, self.path, scan, beam_angles, balance=balance,
                speed=self._speed_est, speed_limit=speed_limit)
        else:
            v, omega = self.fallback.compute(pose, (gx, gy), scan,
                                             beam_angles)
            v, omega = self.walking.clamp_balance(v, omega, balance)

        # ---- goal slowdown and hard stop
        if dist_goal < 1.0:
            scale = max(0.0, (dist_goal - C.GOAL_TOLERANCE)
                        / max(1.0 - C.GOAL_TOLERANCE, 1e-6))
            v *= scale

        # ---- obstacle stop bookkeeping
        blocked = (self.walking.last_clearance <= _HARD_CLEARANCE)
        if blocked and not self._obstacle_active:
            self.stopped_for_obstacle += 1
        self._obstacle_active = bool(blocked)

        # ---- moving-cart guard (warehouse): wait, do not step into the sweep
        cart_gap = self._cart_clearance(pose)
        cart_trigger = 1.15 if self._cart_active else 0.80   # hysteresis
        if cart_gap < cart_trigger:
            if v > 0.0:
                v = 0.0
            if not self._cart_active:
                self.stopped_for_cart += 1
            self._cart_active = True
        else:
            self._cart_active = False

        # ---- stuck detection (replan elsewhere, keep the gait calm)
        if v > 0.15 and self._speed_est < 0.05:
            self._stuck_time += dt
        else:
            self._stuck_time = max(0.0, self._stuck_time - 0.5 * dt)

        self.target_speed = float(v)
        self.target_turn = float(omega)
        return np.array([v, omega], dtype=np.float64)
