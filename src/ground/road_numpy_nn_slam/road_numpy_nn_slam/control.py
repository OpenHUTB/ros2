"""Motion control for the road vehicle: speed PID, Stanley lateral control,
traffic-light and pedestrian rules, and the mission-level controller.

The vehicle pose is ``(x, y, 0, yaw)`` and the simulator command is
``(throttle [-1, 1], steer [-1, 1])`` with ``steer > 0`` meaning "turn left".
:class:`MissionController` returns exactly that pair, so
``sim.step(mission.update(...))`` is the intended usage.

The controller has three safety layers on top of path tracking:

* **traffic lights** -- red (and yellow, when it can still stop) makes the car
  decelerate smoothly to the stop line 5 m before the light, never crossing it;
* **pedestrians** -- a pedestrian inside 12 m ahead / 4 m laterally stops the
  car;
* **frontal clearance** -- lidar distance in a +-40 deg sector limits speed and
  brakes for unexpected obstacles.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .planning import (DWAPlanner, GeometricPlanner, GridPlanner,
                       scene_planning_grid)

__all__ = ["PID", "SpeedController", "Stanley", "SafetyFilter",
           "MissionController"]

_RANGE_MAX = float(C.LIDAR_RANGE_MAX)
_RANGE_MIN = float(C.LIDAR_RANGE_MIN)

# traffic-light stop line is 5 m before the light centre in the simulator
_LIGHT_STOP_OFFSET = 5.0
_LIGHT_STOP_MARGIN = 2.5        # m, aim to halt this far before the stop line
_LIGHT_REACT_DIST = 25.0        # m to the stop line at which we start braking
_COMFORT_DECEL = 1.2            # m/s^2, smooth approach to the lights
_PED_STOP_OFFSET = 5.0          # m, stop this far before a pedestrian on the road
_PED_CAUTION_DIST = 20.0        # m, slow down near any pedestrian close to the road
_PED_CAUTION_SPEED = 4.5        # m/s, speed ceiling inside the caution zone


def _densify_path(path: Sequence[Sequence[float]],
                  step: float = 0.8) -> List[Tuple[float, float]]:
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


class SpeedController:
    """Longitudinal speed PID -> throttle in ``[-1, 1]`` for the road sim.

    The sim accumulates ``a = throttle * MAX_ACCEL`` (or ``throttle *
    MAX_BRAKE`` when braking) minus quadratic drag and rolling resistance, so a
    drag feed-forward keeps the PID from fighting the resistance.  Braking uses
    a larger gain than accelerating: a proportional-only stop would crawl
    towards zero for many seconds instead of stopping at the line.
    """

    def __init__(self, kp: float = 0.70, ki: float = 0.25, kd: float = 0.02,
                 kp_brake: float = 1.20, i_limit: float = 0.6) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.kp_brake = float(kp_brake)
        self.i_limit = float(i_limit)
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error: Optional[float] = None

    def throttle(self, target_speed: float, current_speed: float,
                 dt: float) -> float:
        """Throttle that tracks ``target_speed`` from ``current_speed``."""
        target = max(0.0, float(target_speed))
        current = max(0.0, float(current_speed))
        dt = max(float(dt), 1e-6)
        error = target - current
        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._integral = max(-self.i_limit,
                             min(self.i_limit,
                                 self._integral + error * dt))
        kp = self.kp if error >= 0.0 else self.kp_brake
        net = kp * error + self.ki * self._integral + self.kd * derivative
        net = float(np.clip(net, -C.MAX_BRAKE, C.MAX_ACCEL))
        drag = C.DRAG * current * current + C.ROLLING if current > 1e-3 else 0.0
        if net >= 0.0:
            throttle = (net + drag) / C.MAX_ACCEL
        else:
            throttle = (net + drag) / C.MAX_BRAKE
        return float(np.clip(throttle, -1.0, 1.0))


class Stanley:
    """Stanley lateral controller (heading error + cross-track feedback).

    ``delta = psi_err + atan(k_cross * e / (|v| + k_soft))`` with the
    cross-track error ``e`` positive when the path lies to the left of the
    vehicle, so positive ``delta`` (left steer) reduces it.
    """

    def __init__(self, k_cross: float = 1.8, k_soft: float = 1.5,
                 max_steer: float = C.MAX_STEER) -> None:
        self.k_cross = float(k_cross)
        self.k_soft = float(k_soft)
        self.max_steer = float(max_steer)
        self.last_cross_error = 0.0
        self.last_heading_error = 0.0

    def project(self, pose: Sequence[float], path: Sequence[Sequence[float]],
                index: Optional[int] = None
                ) -> Tuple[int, float, float, float, float, float, float]:
        """Project the pose on the path; return (segment, t, px, py, tangent,
        cross_error, heading_error)."""
        pose = np.asarray(pose, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        if pose.size >= 4:
            yaw = float(pose[3])
        elif pose.size >= 3:
            yaw = float(pose[2])
        else:
            yaw = 0.0
        pts = path
        n = len(pts)
        if n == 1:
            px, py = float(pts[0][0]), float(pts[0][1])
            return 0, 0.0, px, py, yaw, 0.0, 0.0
        if index is None:
            lo, hi = 0, n - 1
        else:
            lo = max(0, int(index) - 3)
            hi = min(n - 1, int(index) + 20)
        best = (math.inf, lo, 0.0)
        for j in range(lo, hi):
            ax, ay = float(pts[j][0]), float(pts[j][1])
            bx, by = float(pts[j + 1][0]), float(pts[j + 1][1])
            abx, aby = bx - ax, by - ay
            seg2 = abx * abx + aby * aby
            t = 0.0 if seg2 <= 1e-12 else \
                max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / seg2))
            dx, dy = ax + t * abx - x, ay + t * aby - y
            d2 = dx * dx + dy * dy
            if d2 < best[0]:
                best = (d2, j, t)
        _, j, t = best
        ax, ay = float(pts[j][0]), float(pts[j][1])
        bx, by = float(pts[j + 1][0]), float(pts[j + 1][1])
        tx, ty = bx - ax, by - ay
        seg = math.hypot(tx, ty)
        if seg <= 1e-9:
            tan_yaw = yaw
        else:
            tan_yaw = math.atan2(ty, tx)
        px, py = ax + t * tx, ay + t * ty
        dx, dy = px - x, py - y
        # positive when the path is to the left of the car
        cross = -math.sin(yaw) * dx + math.cos(yaw) * dy
        heading = wrap_angle(tan_yaw - yaw)
        return j, t, px, py, tan_yaw, cross, heading

    def compute(self, pose: Sequence[float], path: Sequence[Sequence[float]],
                speed: float = 0.0,
                index: Optional[int] = None) -> Tuple[float, float, float]:
        """Return ``(steer_angle_rad, cross_error_m, heading_error_rad)``."""
        if path is None or len(path) == 0:
            return 0.0, 0.0, 0.0
        _, _, _, _, _, cross, heading = self.project(pose, path, index)
        speed = abs(float(speed))
        delta = heading + math.atan2(self.k_cross * cross,
                                     speed + self.k_soft)
        self.last_cross_error = float(cross)
        self.last_heading_error = float(heading)
        return float(np.clip(delta, -self.max_steer, self.max_steer)), \
            float(cross), float(heading)


class SafetyFilter:
    """Frontal lidar safety filter for the road vehicle.

    ``speed_limit`` returns a speed ceiling from the clearance in a +-40 deg
    sector; ``apply`` also caps a raw ``(throttle, steer)`` command, which keeps
    the name used by the shared ROS wrappers working.
    """

    def __init__(self, hard_clearance: float = 1.6,
                 soft_clearance: float = 3.5,
                 half_fov_deg: float = 40.0) -> None:
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

    def speed_limit(self, scan: Sequence[float], beam_angles: Sequence[float],
                    cruise: float = C.MAX_SPEED,
                    center_angle: float = 0.0) -> float:
        clearance = self.frontal_clearance(scan, beam_angles, center_angle)
        if clearance <= self.hard_clearance:
            return 0.0
        if clearance >= self.soft_clearance:
            return float(cruise)
        span = max(self.soft_clearance - self.hard_clearance, 1e-6)
        return float(cruise) * (clearance - self.hard_clearance) / span

    def apply(self, cmd: Sequence[float], scan: Sequence[float],
              beam_angles: Sequence[float],
              cruise: float = C.MAX_SPEED) -> np.ndarray:
        """Cap a ``(throttle, steer)`` command for the frontal clearance."""
        out = np.array(cmd, dtype=np.float64, copy=True)
        limit = self.speed_limit(scan, beam_angles, cruise)
        if limit <= 1e-6:
            out[0] = -1.0
        elif limit < cruise:
            out[0] = min(out[0], max(limit / max(cruise, 1e-6), 0.0))
        return out


class MissionController:
    """Mission-level road-vehicle controller.

    Global A* on the road occupancy grid (off-road and obstacles blocked) plus
    Stanley path tracking, a longitudinal speed PID, traffic-light stopping and
    pedestrian yielding.

    Parameters
    ----------
    scene_or_map:
        :class:`scenes.Scene` (its road occupancy is used) or a 2-D boolean
        blocked grid.
    resolution:
        Planning grid resolution in metres, default
        :data:`config.PLAN_RESOLUTION`.
    replan_interval:
        Maximum time (s) between re-plans; the path is also re-planned when it
        becomes invalid or the goal moves.
    cruise_speed:
        Desired free-flow speed (m/s); the traffic-light, pedestrian, corner
        and clearance rules reduce it locally.
    goal_tolerance:
        Distance (m) to the goal inside which the car brakes to a stop.
    """

    def __init__(self, scene_or_map, resolution: float = C.PLAN_RESOLUTION,
                 replan_interval: float = 10.0,
                 cruise_speed: float = min(C.MAX_SPEED, 8.0),
                 goal_tolerance: float = 2.5) -> None:
        scene = None
        if hasattr(scene_or_map, "occupancy") or hasattr(scene_or_map, "coarse_map"):
            scene = scene_or_map
            base = scene_planning_grid(scene, resolution)
        else:
            base = np.asarray(scene_or_map, dtype=bool)
        self.scene = scene
        self.resolution = float(resolution)
        self.cruise_speed = float(min(cruise_speed, C.MAX_SPEED))
        self.goal_tolerance = float(goal_tolerance)
        self._base_grid = base
        # multi-level inflation: wide safety margin first, fall back only when a
        # scene is too narrow for it
        self._radii = (max(C.CAR_RADIUS, C.INFLATION_RADIUS),
                       max(C.CAR_RADIUS + 0.30, C.INFLATION_RADIUS - 0.25),
                       C.CAR_RADIUS + 0.03)
        self._planners: List[GridPlanner] = []
        self.planner = self._get_planner(0)
        self.stanley = Stanley()
        self.speed_ctrl = SpeedController()
        self.safety = SafetyFilter()
        self.fallback = GeometricPlanner(scene=scene)
        self.replan_interval = float(replan_interval)
        self.path: List[Tuple[float, float]] = []
        self.replans = 0
        self.stopped_for_light = 0
        self.stopped_for_pedestrian = 0
        self.yields_for_light = 0
        self.yields_for_pedestrian = 0
        self.target_speed = 0.0
        self.steer_angle = 0.0
        self._v_profile: List[float] = []
        self._goal: Optional[Tuple[float, float]] = None
        self._timer = 0.0
        self._blocked_time = 0.0
        self._time = 0.0
        self._speed_est = 0.0
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._light_active = False
        self._ped_active = False
        self._light_yield_active = False
        self._ped_yield_active = False

    # ---------------------------------------------------------------- helpers
    def reset(self, pose: Optional[Sequence[float]] = None) -> None:
        """Reset the per-episode state (path, counters and internal clock).

        Closed-loop harnesses call ``controller.reset(...)`` before every
        episode.  The internal clock drives the traffic-light and pedestrian
        lookups whenever the caller does not pass them explicitly, so it has to
        restart with the episode as well.
        """
        self.path = []
        self._v_profile = []
        self.replans = 0
        self.stopped_for_light = 0
        self.stopped_for_pedestrian = 0
        self.yields_for_light = 0
        self.yields_for_pedestrian = 0
        self.target_speed = 0.0
        self.steer_angle = 0.0
        self._goal = None
        self._timer = 0.0
        self._blocked_time = 0.0
        self._time = 0.0
        self._speed_est = 0.0
        self.speed_ctrl.reset()
        self._light_active = False
        self._ped_active = False
        self._light_yield_active = False
        self._ped_yield_active = False
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

    def _smooth_path(self, pts: Sequence[Sequence[float]],
                     iterations: int = 40,
                     weight: float = 0.5) -> List[Tuple[float, float]]:
        """Round the corners of a dense path while keeping it in free space.

        A pure line-of-sight A* path can contain kinks that an Ackermann vehicle
        cannot follow; a few Laplacian smoothing sweeps spread the heading
        change over several metres.  Candidate points that would leave the
        inflated free space are rejected, so the safety margin is preserved.
        """
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

    def _build_profile(self) -> None:
        """Per-waypoint speed ceiling from path clearance and curvature."""
        pts = self.path
        n = len(pts)
        prof: List[float] = []
        car = float(C.CAR_RADIUS)
        for i in range(n):
            v = self.cruise_speed
            if n >= 2:
                c = self.planner.clearance(pts[i][0], pts[i][1])
                if c < car + 0.85:
                    v = min(v, 3.0)
                elif c < car + 1.30:
                    v = min(v, 4.0)
                elif c < car + 2.00:
                    v = min(v, 5.5)
            if 0 < i < n - 1:
                a1 = math.atan2(pts[i][1] - pts[i - 1][1],
                                pts[i][0] - pts[i - 1][0])
                a2 = math.atan2(pts[i + 1][1] - pts[i][1],
                                pts[i + 1][0] - pts[i][0])
                turn = abs(wrap_angle(a2 - a1))
                if turn > 0.6:
                    v = min(v, 3.5)
                elif turn > 0.25:
                    v = min(v, 6.0)
            prof.append(float(v))
        self._v_profile = prof

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
        self.path = self._smooth_path(_densify_path(path)) if path else []
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._timer = self.replan_interval
        self._blocked_time = 0.0
        self.replans += 1
        if self.path:
            self._build_profile()
        else:
            self._v_profile = []

    def _needs_replan(self, pose_xy: Tuple[float, float],
                      goal_xy: Tuple[float, float]) -> bool:
        if self._goal is None or not self.path:
            return True
        if math.hypot(goal_xy[0] - self._goal[0],
                      goal_xy[1] - self._goal[1]) > 0.5:
            return True
        if math.hypot(goal_xy[0] - pose_xy[0],
                      goal_xy[1] - pose_xy[1]) < max(4.0,
                                                     self.goal_tolerance + 1.0):
            return False
        if self._blocked_time > 2.0:
            return True
        if self._timer <= 0.0:
            return True
        n = len(self.path)
        for k in range(0, n, 3):
            x, y = self.path[k]
            if not self.planner.is_free(x, y):
                return True
        return False

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

    # -------------------------------------------------------------- light/ped
    def _light_limit(self, light: Optional[Dict[str, float]]
                     ) -> Tuple[Optional[float], bool]:
        """Allowed speed (m/s) and "stopping" flag for a traffic light dict."""
        if not light:
            return None, False
        state = str(light.get("state", "green"))
        if state == "green":
            return None, False
        dist = float(light.get("distance", 1e9))
        stop_line = dist - _LIGHT_STOP_OFFSET
        if stop_line > _LIGHT_REACT_DIST:
            return None, False
        if stop_line < 0.2:
            # already on/over the line: clear the junction instead of stopping
            return None, False
        if state == "yellow":
            # crossing on yellow is legal if we cannot stop comfortably
            need = self._speed_est * self._speed_est / (2.0 * C.MAX_BRAKE)
            if need > max(stop_line - 0.5, 0.0) + 0.5:
                return None, False
        gap = max(0.0, stop_line - _LIGHT_STOP_MARGIN)
        if gap <= 1e-6:
            return 0.0, True
        v_allow = math.sqrt(2.0 * _COMFORT_DECEL * gap + 1e-9)
        return float(min(v_allow, self.cruise_speed)), bool(v_allow < 0.6)

    def _ped_limit(self, pedestrians: Optional[Sequence[Sequence[float]]],
                   pose: np.ndarray) -> Tuple[Optional[float], bool]:
        """Allowed speed (m/s) and "stopping" flag for pedestrians ahead.

        A pedestrian inside the driving corridor (12 m ahead, 4.5 m laterally)
        is treated as a stop; a pedestrian close to the road anywhere within
        20 m also limits the speed so the car can still stop if it steps in.
        """
        if not pedestrians:
            return None, False
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_allow: Optional[float] = None
        stopping = False
        caution = False
        for ped in pedestrians:
            px, py = float(ped[0]), float(ped[1])
            dx, dy = px - x, py - y
            forward = dx * cy + dy * sy
            lateral = -dx * sy + dy * cy
            if forward > _PED_CAUTION_DIST or forward < -2.0:
                continue
            if abs(lateral) <= 10.0:
                caution = True
            if forward <= 0.5:
                # beside or behind the car: braking cannot help, drive away
                continue
            if abs(lateral) > 4.0 or forward > 12.0:
                continue
            if forward <= _PED_STOP_OFFSET:
                # pedestrian inside the safety gap ahead: hard stop
                return 0.0, True
            gap = max(0.0, forward - _PED_STOP_OFFSET)
            v = min(math.sqrt(2.0 * _COMFORT_DECEL * gap + 1e-9),
                    self.cruise_speed)
            v_allow = v if v_allow is None else min(v_allow, v)
            stopping = stopping or (v < 0.8)
        if caution:
            v_allow = _PED_CAUTION_SPEED if v_allow is None else \
                min(v_allow, _PED_CAUTION_SPEED)
        return (None if v_allow is None else float(v_allow)), bool(stopping)

    # ----------------------------------------------------------------- update
    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan: Sequence[float], beam_angles: Sequence[float],
               dt: float,
               light: Optional[Dict[str, float]] = None,
               pedestrians: Optional[Sequence[Sequence[float]]] = None
               ) -> np.ndarray:
        """Compute one ``(throttle, steer)`` command for ``RoadSim.step``.

        ``goal`` is ``[x, y]`` or ``[x, y, 0]``; ``light`` is a traffic-light
        dict as returned by :meth:`sim.RoadSim.traffic_light` and
        ``pedestrians`` a list of ``(x, y)`` positions.  When omitted they are
        derived from the scene using the controller's own clock.
        """
        pose = np.asarray(pose, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        dt = max(float(dt), 1e-6)
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        gx = float(goal[0])
        gy = float(goal[1])

        # speed estimate from consecutive poses (the mission API is pose-only)
        if self._prev_xy is None:
            self._speed_est = 0.0
        else:
            meas = math.hypot(x - self._prev_xy[0],
                              y - self._prev_xy[1]) / dt
            meas = min(meas, C.MAX_SPEED + 3.0)
            self._speed_est = 0.6 * self._speed_est + 0.4 * meas
        self._prev_xy = (x, y)
        self._time += dt
        self._timer -= dt

        if light is not None and not isinstance(light, dict):
            # tolerate harnesses that put a timestamp/scalar in the light slot
            light = None
        if pedestrians is not None and not isinstance(
                pedestrians, (list, tuple, np.ndarray)):
            pedestrians = None
        if light is None and self.scene is not None:
            light = self.scene.nearest_light(x, y, yaw, self._time)
        if pedestrians is None and self.scene is not None:
            pedestrians = self.scene.pedestrian_positions(self._time)

        dist_goal = math.hypot(gx - x, gy - y)
        if self._needs_replan((x, y), (gx, gy)):
            self._replan(pose, (gx, gy))

        # ---- lateral control and free-flow speed
        if self.path and len(self.path) >= 2:
            index = self._closest_index(x, y)
            steer_angle, _, _ = self.stanley.compute(
                pose, self.path, self._speed_est, index=index)
            v_target = min(self.cruise_speed,
                           self._profile_limit(index, 20.0))
        else:
            steer_angle = 0.0
            v_target = self.cruise_speed * 0.4

        # ---- goal approach
        if dist_goal <= self.goal_tolerance:
            v_target = 0.0
        else:
            gap = dist_goal - max(self.goal_tolerance - 0.5, 0.0)
            if gap < 12.0:
                v_target = min(v_target,
                               max(1.0, math.sqrt(2.0 * 1.5 * gap)))

        # ---- traffic lights, pedestrians and frontal clearance
        v_light, light_stop = self._light_limit(light)
        if v_light is not None:
            v_target = min(v_target, v_light)
        v_ped, ped_stop = self._ped_limit(pedestrians, pose)
        if v_ped is not None:
            v_target = min(v_target, v_ped)
        v_safe = self.safety.speed_limit(scan, beam_angles,
                                         self.cruise_speed)
        v_target = min(v_target, v_safe)

        # count "yield" events (a red/yellow light or a pedestrian reduced the
        # desired speed) separately from full stops
        light_yield = v_light is not None and v_light < self.cruise_speed - 0.1
        if light_yield and not self._light_yield_active:
            self.yields_for_light += 1
        self._light_yield_active = bool(light_yield)
        ped_yield = v_ped is not None and v_ped < self.cruise_speed - 0.1
        if ped_yield and not self._ped_yield_active:
            self.yields_for_pedestrian += 1
        self._ped_yield_active = bool(ped_yield)

        # ---- longitudinal command
        throttle = self.speed_ctrl.throttle(v_target, self._speed_est, dt)
        if v_target <= 0.05 and self._speed_est < 0.3:
            throttle = -1.0        # hold the brake while stopped
        steer_norm = float(np.clip(steer_angle / C.MAX_STEER, -1.0, 1.0))

        # ---- bookkeeping
        if light_stop and not self._light_active:
            self.stopped_for_light += 1
        self._light_active = bool(light_stop)
        if ped_stop and not self._ped_active:
            self.stopped_for_pedestrian += 1
        self._ped_active = bool(ped_stop)
        if v_target > 1.0 and self._speed_est < 0.3:
            self._blocked_time += dt
        else:
            self._blocked_time = 0.0
        self.target_speed = float(v_target)
        self.steer_angle = float(steer_angle)
        return np.array([throttle, steer_norm], dtype=np.float64)


# Legacy names from the shared vehicle templates: the road vehicle has neither a
# depth nor an altitude loop, so its longitudinal speed controller takes on
# those roles for code written against the generic template.
DepthController = SpeedController
AltitudeController = SpeedController
