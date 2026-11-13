"""Motion control for the UAV: PID, altitude hold, pure pursuit, safety filter
and the mission-level controller.

The vehicle pose is ``(x, y, altitude, yaw)`` with **altitude positive up** and
the command is ``(vx, vy, vz, yaw_rate)`` with ``vz`` positive up.  ``UAVSim``
tracks the horizontal command in the **world frame** (its outer loop compares
``cmd[0:2]`` against the world-frame linear velocity), so
:class:`MissionController` by default emits world-frame ``[vx, vy, vz, yaw_rate]``
which can be fed to :meth:`sim.UAVSim.step` directly.  Construct it with
``world_frame=False`` to get the body-frame ``[surge, 0, vz, yaw_rate]`` shape.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .planning import GeometricPlanner, GridPlanner, nfz_margin_mask

__all__ = ["PID", "DepthController", "AltitudeController", "PurePursuit",
           "SafetyFilter", "MissionController"]

_RANGE_MAX = float(C.LIDAR_RANGE_MAX)
_RANGE_MIN = float(C.LIDAR_RANGE_MIN)


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


class DepthController:
    """Altitude hold controller (historical ROV name kept for compatibility).

    ``current_depth`` is now the **altitude, positive up** (the ROV depth was
    positive down), so a positive error "we are too low" produces a positive
    ``vz`` (climb), clamped to ``+/- config.MAX_VZ``.  The output is a velocity
    command for the sim's inner velocity loop, not a thrust.
    """

    def __init__(self, target: Optional[float] = None,
                 kp: float = 0.85, ki: float = 0.05, kd: float = 0.16) -> None:
        self.target: Optional[float] = None if target is None else float(target)
        self.pid = PID(kp=kp, ki=ki, kd=kd, limit=C.MAX_VZ, i_limit=0.5)

    def reset(self) -> None:
        self.pid.reset()

    def update(self, current_depth: float, target_depth: float,
               dt: float) -> float:
        """Altitude error (m) -> climb rate command (m/s, positive up)."""
        self.target = float(target_depth)
        altitude = float(current_depth)
        error = self.target - altitude
        vz = self.pid.update(error, dt)
        vz = float(np.clip(vz, -C.MAX_VZ, C.MAX_VZ))
        # soft altitude envelope so the vehicle cannot be commanded out of the
        # certified flight band
        if altitude <= C.MIN_ALT and vz < 0.0:
            vz = 0.0
        if altitude >= C.MAX_ALT and vz > 0.0:
            vz = 0.0
        return vz


# Aerial naming for the retargeted stack.
AltitudeController = DepthController


class PurePursuit:
    """Lookahead path tracker for a velocity-controlled aerial vehicle.

    ``compute`` keeps the classic signature and returns body-frame
    ``(forward_speed, yaw_rate)``.  ``compute_world`` returns the world-frame
    velocity drive ``(vx, vy, yaw_rate)`` used by :class:`MissionController`.
    """

    def __init__(self, lookahead: float = 2.6,
                 max_surge: float = 6.0,
                 max_yaw: float = C.MAX_YAW_RATE,
                 goal_tolerance: float = 0.7) -> None:
        self.lookahead = float(lookahead)
        self.max_surge = float(max_surge)
        self.max_yaw = float(max_yaw)
        self.goal_tolerance = float(goal_tolerance)
        self.yaw_gain = 1.8
        self.speed_floor = 0.22
        self.endgame = 2.5

    # --------------------------------------------------------------- helpers
    def _lookahead_target(self, pose: np.ndarray,
                          pts: np.ndarray) -> Tuple[float, float, float]:
        """Return ``(target_x, target_y, distance_to_path_end)``."""
        x, y = float(pose[0]), float(pose[1])
        ex, ey = float(pts[-1, 0]), float(pts[-1, 1])
        d_end = math.hypot(ex - x, ey - y)
        if pts.shape[0] == 1:
            return ex, ey, d_end
        # project the UAV onto the closest path segment
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
        # walk along the polyline until the lookahead arc length is reached
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

    def _speed(self, alpha: float, d_end: float) -> float:
        align = max(0.0, math.cos(alpha))
        speed = self.max_surge * (self.speed_floor
                                  + (1.0 - self.speed_floor) * align)
        if d_end < self.endgame:
            speed *= 0.35 + 0.65 * (d_end / self.endgame)
        return speed

    # ---------------------------------------------------------------- public
    def compute(self, pose_xy_yaw: Sequence[float],
                path: Optional[Sequence[Sequence[float]]]) -> Tuple[float, float]:
        """Body-frame control ``(forward_speed, yaw_rate)``."""
        if path is None or len(path) < 1:
            return 0.0, 0.0
        pose = np.asarray(pose_xy_yaw, dtype=np.float64).ravel()
        pts = np.asarray(path, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0:
            return 0.0, 0.0
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if pose.size >= 3 else 0.0
        tx, ty, d_end = self._lookahead_target(pose, pts)
        if d_end < self.goal_tolerance:
            return 0.0, 0.0
        alpha = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        wz = float(np.clip(self.yaw_gain * alpha, -self.max_yaw, self.max_yaw))
        return float(self._speed(alpha, d_end)), wz

    def compute_world(self, pose_xy_yaw: Sequence[float],
                      path: Optional[Sequence[Sequence[float]]]
                      ) -> Tuple[float, float, float]:
        """World-frame drive ``(vx, vy, yaw_rate)``."""
        if path is None or len(path) < 1:
            return 0.0, 0.0, 0.0
        pose = np.asarray(pose_xy_yaw, dtype=np.float64).ravel()
        pts = np.asarray(path, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0:
            return 0.0, 0.0, 0.0
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if pose.size >= 3 else 0.0
        tx, ty, d_end = self._lookahead_target(pose, pts)
        if d_end < self.goal_tolerance:
            return 0.0, 0.0, 0.0
        bearing = math.atan2(ty - y, tx - x)
        alpha = wrap_angle(bearing - yaw)
        wz = float(np.clip(self.yaw_gain * alpha, -self.max_yaw, self.max_yaw))
        speed = self._speed(alpha, d_end)
        return (float(speed * math.cos(bearing)),
                float(speed * math.sin(bearing)), wz)


class SafetyFilter:
    """Forward lidar safety filter: slow down / stop for low frontal clearance.

    The physical UAV radius is 1.2 m, so the soft/hard thresholds are markedly
    larger than the ROV values (the vehicle cannot stop in a few centimetres).
    """

    def __init__(self, min_clearance: float = 3.2,
                 hard_clearance: float = 1.6,
                 half_fov_deg: float = 55.0) -> None:
        self.min_clearance = float(min_clearance)
        self.hard_clearance = float(hard_clearance)
        self.half_fov = math.radians(float(half_fov_deg))

    def frontal_clearance(self, scan: Sequence[float],
                          beam_angles: Sequence[float],
                          center_angle: float = 0.0) -> float:
        """Minimum forward lidar range inside the safety sector (m).

        ``center_angle`` rotates the sector away from the heading, which lets
        the world-frame guidance check the clearance along the actual travel
        direction.
        """
        ranges = np.asarray(scan, dtype=np.float64).ravel()
        beams = np.asarray(beam_angles, dtype=np.float64).ravel()
        n = min(ranges.size, beams.size)
        if n == 0:
            return _RANGE_MAX
        rel = np.abs(np.asarray([wrap_angle(b - float(center_angle))
                                 for b in beams[:n]], dtype=np.float64))
        front = rel <= self.half_fov
        valid = (np.isfinite(ranges[:n]) & (ranges[:n] < _RANGE_MAX * 0.995)
                 & (ranges[:n] > _RANGE_MIN + 0.02))
        sel = front & valid
        if not np.any(sel):
            return _RANGE_MAX
        return float(np.min(ranges[:n][sel]))

    def speed_scale(self, scan: Sequence[float], beam_angles: Sequence[float],
                    center_angle: float = 0.0) -> float:
        """Multiplicative speed limit in ``[0, 1]`` from the frontal clearance."""
        clearance = self.frontal_clearance(scan, beam_angles, center_angle)
        if clearance <= self.hard_clearance:
            return 0.0
        if clearance >= self.min_clearance:
            return 1.0
        span = max(self.min_clearance - self.hard_clearance, 1e-6)
        scale = (clearance - self.hard_clearance) / span
        return float(0.10 + 0.90 * max(0.0, min(1.0, scale)))

    def apply(self, cmd: Sequence[float], scan: Sequence[float],
              beam_angles: Sequence[float]) -> np.ndarray:
        """Return ``cmd`` with the forward speed reduced for the clearance."""
        out = np.array(cmd, dtype=np.float64, copy=True)
        scale = self.speed_scale(scan, beam_angles)
        out[0] = max(0.0, out[0]) * scale
        return out


class MissionController:
    """Mission-level controller: A* + pure pursuit + altitude hold + safety.

    The planning grid is built from :meth:`scenes.Scene.planning_map` (buildings
    **and** no-fly zones are blocked) and additionally keeps an ``nfz_margin``
    stand-off from the no-fly zones; obstacles are inflated by the UAV radius.
    A hard no-fly-zone safety rule then steers the vehicle away whenever the
    signed clearance drops below :attr:`nfz_soft_margin` (6 m), as a safety net
    for wind drift and tracking error.

    Parameters
    ----------
    scene_or_map:
        :class:`scenes.Scene` or a 2-D boolean blocked grid.
    resolution:
        Planning grid resolution (m), default :data:`config.PLAN_RESOLUTION`.
    replan_interval:
        Maximum time (s) between re-plans (the path is also re-planned when it
        becomes invalid or the goal moves).
    world_frame:
        ``True`` (default) -> return world-frame ``[vx, vy, vz, yaw_rate]`` for
        :meth:`sim.UAVSim.step`; ``False`` -> body-frame
        ``[forward_speed, 0, vz, yaw_rate]``.
    nfz_margin:
        Extra planning stand-off around no-fly zones (m).
    """

    def __init__(self, scene_or_map, resolution: float = C.PLAN_RESOLUTION,
                 replan_interval: float = 15.0, world_frame: bool = True,
                 nfz_margin: float = 4.0) -> None:
        scene = None
        if hasattr(scene_or_map, "planning_map"):
            scene = scene_or_map
            base = np.asarray(scene.planning_map(resolution), dtype=bool)
            if getattr(scene, "nfz", None):
                base = base | nfz_margin_mask(scene, resolution, nfz_margin)
        elif hasattr(scene_or_map, "coarse_map"):
            base = np.asarray(scene_or_map.coarse_map(resolution), dtype=bool)
        else:
            base = np.asarray(scene_or_map, dtype=bool)

        self.scene = scene
        self.resolution = float(resolution)
        self.world_frame = bool(world_frame)
        self.uav_radius = float(C.UAV_RADIUS)
        # multi-level inflation: prefer the wide safety margin, fall back to
        # tighter radii only when a scene is too narrow for the wide one.
        # Planners are built lazily: the wide one covers every shipped scene, so
        # the tighter fall-backs usually cost nothing at all.
        self._base_grid = base
        self._radii = (C.UAV_RADIUS + 0.60, C.UAV_RADIUS + 0.30,
                       C.UAV_RADIUS + 0.03)
        self._planners: List[GridPlanner] = []
        self.planner = self._get_planner(0)
        self.pursuit = PurePursuit()
        self.alt_hold = DepthController()
        self.depth = self.alt_hold          # ROV-compatible alias
        self.safety = SafetyFilter()
        self.fallback = GeometricPlanner(max_surge=6.0, scene=scene)
        self.replan_interval = float(replan_interval)
        self.nfz_soft_margin = 6.0
        self.nfz_push = 3.0
        self.nfz_yaw_gain = 2.2
        self.goal_xy_tolerance = 0.9
        self.path: List[Tuple[float, float]] = []
        self.replans = 0
        self.last_speed = 0.0
        self._goal: Optional[Tuple[float, float]] = None
        self._timer = 0.0
        self._blocked_time = 0.0

    # ---------------------------------------------------------------- helpers
    def _get_planner(self, index: int) -> GridPlanner:
        """Lazily build the ``index``-th inflation level of the A* planner."""
        while len(self._planners) <= index:
            radius = self._radii[min(index, len(self._radii) - 1)]
            self._planners.append(GridPlanner(
                self._base_grid, self.resolution, radius / self.resolution))
        return self._planners[index]

    def _replan(self, pose: Sequence[float],
                goal_xy: Tuple[float, float]) -> None:
        path = None
        for index in range(len(self._radii)):
            planner = self._get_planner(index)
            path = planner.plan((float(pose[0]), float(pose[1])), goal_xy)
            if path:
                self.planner = planner
                break
        self.path = list(path) if path else []
        self._goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._timer = self.replan_interval
        self._blocked_time = 0.0
        self.replans += 1

    def _needs_replan(self, pose_xy: Tuple[float, float],
                      goal_xy: Tuple[float, float]) -> bool:
        if self._goal is None or not self.path:
            return True
        if math.hypot(goal_xy[0] - self._goal[0],
                      goal_xy[1] - self._goal[1]) > 0.5:
            return True
        # close to the goal the current path is good enough; skip the search
        if math.hypot(goal_xy[0] - pose_xy[0],
                      goal_xy[1] - pose_xy[1]) < 3.0:
            return False
        if self._timer <= 0.0:
            return True
        if self._blocked_time > 1.2:
            return True
        # existing path invalidated by a newly observed obstacle
        n = len(self.path)
        for k in range(0, n, 4):
            x, y = self.path[k]
            if not self.planner.is_free(x, y):
                return True
        return False
    # ------------------------------------------------------------- NFZ guard
    def _nfz_guard(self, vx: float, vy: float, wz: float,
                   pose: np.ndarray) -> Tuple[float, float, float]:
        """Hard no-fly-zone safety net: steer/translate away below 6 m."""
        scene = self.scene
        if scene is None or not getattr(scene, "nfz", None):
            return vx, vy, wz
        x, y = float(pose[0]), float(pose[1])
        clearance = scene.nfz_clearance(x, y)
        if clearance >= self.nfz_soft_margin:
            return vx, vy, wz
        best = min(scene.nfz,
                   key=lambda z: math.hypot(x - z[0], y - z[1]) - z[2])
        cx, cy = float(best[0]), float(best[1])
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            dx, dy, dist = 1.0, 0.0, 1.0
        ox, oy = dx / dist, dy / dist          # outward normal
        w = float(np.clip((self.nfz_soft_margin - clearance)
                          / self.nfz_soft_margin, 0.0, 1.0))
        if clearance < 0.0:
            vx *= 0.3
            vy *= 0.3
        # cancel the velocity component that points into the zone
        inward = vx * ox + vy * oy
        if inward < 0.0:
            vx -= ox * inward
            vy -= oy * inward
        # push out along the outward normal
        push = self.nfz_push * max(w, 1.0 if clearance < 0.0 else 0.0)
        vx += ox * push
        vy += oy * push
        # turn the nose towards the escape direction as well
        target_yaw = math.atan2(oy, ox)
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        alpha = wrap_angle(target_yaw - yaw)
        wz = float(np.clip((1.0 - w) * wz + w * self.nfz_yaw_gain * alpha,
                           -C.MAX_YAW_RATE, C.MAX_YAW_RATE))
        speed = math.hypot(vx, vy)
        if speed > C.MAX_VX:
            vx *= C.MAX_VX / speed
            vy *= C.MAX_VX / speed
        return vx, vy, wz

    # ----------------------------------------------------------------- update
    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan: Sequence[float], beam_angles: Sequence[float],
               dt: float) -> np.ndarray:
        """Compute one control command.

        Returns ``[vx, vy, vz, yaw_rate]`` in the world frame (default), where
        ``vz`` is positive up and ``goal`` is ``[x, y, altitude]``.
        """
        pose = np.asarray(pose, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        dt = max(float(dt), 1e-6)
        self._timer -= dt
        x = float(pose[0])
        y = float(pose[1])
        alt = float(pose[2]) if pose.size > 2 else float(C.DEFAULT_ALT)
        yaw = float(pose[3]) if pose.size >= 4 else 0.0
        gx = float(goal[0])
        gy = float(goal[1])
        gz = float(goal[2]) if goal.size > 2 else float(C.DEFAULT_ALT)
        dist_goal = math.hypot(gx - x, gy - y)

        if self._needs_replan((x, y), (gx, gy)):
            self._replan(pose, (gx, gy))

        if dist_goal <= self.goal_xy_tolerance:
            vx, vy, wz = 0.0, 0.0, 0.0
        elif self.path:
            vx, vy, wz = self.pursuit.compute_world([x, y, yaw], self.path)
        else:
            surge, wz = self.fallback.compute([x, y, alt, yaw], (gx, gy),
                                              scan, beam_angles)
            vx = surge * math.cos(yaw)
            vy = surge * math.sin(yaw)

        # frontal-obstacle safety filter along the travel direction
        travel = math.atan2(vy, vx) if (abs(vx) + abs(vy)) > 1e-6 else yaw
        scale = self.safety.speed_scale(scan, beam_angles,
                                        center_angle=wrap_angle(travel - yaw))
        vx *= scale
        vy *= scale

        vz = self.alt_hold.update(alt, gz, dt)

        # no-fly-zone safety net
        vx, vy, wz = self._nfz_guard(vx, vy, wz, pose)

        self.last_speed = math.hypot(vx, vy)
        if scale < 0.15 and dist_goal > self.goal_xy_tolerance:
            self._blocked_time += dt
        else:
            self._blocked_time = 0.0

        if self.world_frame:
            return np.array([vx, vy, vz, wz], dtype=np.float64)
        surge = vx * math.cos(yaw) + vy * math.sin(yaw)
        return np.array([surge, 0.0, vz, wz], dtype=np.float64)
