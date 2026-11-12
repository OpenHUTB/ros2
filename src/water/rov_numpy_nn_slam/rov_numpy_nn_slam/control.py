"""运动控制：PID、定深、纯跟踪、安全滤波与任务级控制器。"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle
from .planning import GeometricPlanner, GridPlanner

__all__ = ["PID", "DepthController", "PurePursuit", "SafetyFilter",
           "MissionController"]


class PID:
    """带输出限幅与积分限幅的离散 PID 控制器。"""

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
    """定深控制：深度误差 -> heave（正值上浮，负值下潜），限幅 ±MAX_HEAVE。"""

    def __init__(self, target: Optional[float] = None) -> None:
        self.target: Optional[float] = None if target is None else float(target)
        self.pid = PID(kp=0.9, ki=0.12, kd=0.30,
                       limit=C.MAX_HEAVE, i_limit=0.35)

    def reset(self) -> None:
        self.pid.reset()

    def update(self, current_depth: float, target_depth: float, dt: float) -> float:
        self.target = float(target_depth)
        # depth 向下为正；误差为正说明当前比目标更深，需要上浮（heave > 0）
        error = float(current_depth) - self.target
        heave = self.pid.update(error, dt)
        return float(np.clip(heave, -C.MAX_HEAVE, C.MAX_HEAVE))


class PurePursuit:
    """前视点纯跟踪横向控制器，输出 ``(surge, yaw_rate)``。"""

    def __init__(self, lookahead: float = 0.9,
                 max_surge: float = C.MAX_SURGE,
                 max_yaw: float = C.MAX_YAW_RATE,
                 goal_tolerance: float = 0.35) -> None:
        self.lookahead = float(lookahead)
        self.max_surge = float(max_surge)
        self.max_yaw = float(max_yaw)
        self.goal_tolerance = float(goal_tolerance)
        self.yaw_gain = 2.2

    def compute(self, pose_xy_yaw: Sequence[float],
                path: Optional[Sequence[Sequence[float]]]) -> Tuple[float, float]:
        if path is None or len(path) < 1:
            return 0.0, 0.0
        pose = np.asarray(pose_xy_yaw, dtype=np.float64).ravel()
        x, y = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if pose.size >= 3 else 0.0
        pts = np.asarray(path, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] == 0:
            return 0.0, 0.0
        ex, ey = float(pts[-1, 0]), float(pts[-1, 1])
        d_end = math.hypot(ex - x, ey - y)
        if d_end < self.goal_tolerance:
            return 0.0, 0.0

        if pts.shape[0] == 1:
            target = (ex, ey)
        else:
            # project the robot onto the closest path segment
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
            # walk along the polyline until the lookahead arc length is reached
            target = (ex, ey)
            remain = (1.0 - t0) * seg_len
            if remain >= self.lookahead:
                ratio = self.lookahead / max(seg_len, 1e-9)
                target = (ax + (t0 + ratio) * (bx - ax),
                          ay + (t0 + ratio) * (by - ay))
            else:
                for j in range(j0 + 1, pts.shape[0] - 1):
                    sx, sy = pts[j]
                    nx, ny = pts[j + 1]
                    seg_len = math.hypot(nx - sx, ny - sy)
                    if remain + seg_len >= self.lookahead:
                        ratio = (self.lookahead - remain) / max(seg_len, 1e-9)
                        target = (sx + ratio * (nx - sx), sy + ratio * (ny - sy))
                        break
                    remain += seg_len

        alpha = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        wz = float(np.clip(self.yaw_gain * alpha, -self.max_yaw, self.max_yaw))
        align = max(0.0, math.cos(alpha))
        surge = self.max_surge * (0.25 + 0.75 * align)
        if d_end < 1.5:
            surge *= 0.35 + 0.65 * (d_end / 1.5)
        if abs(alpha) > 1.3:
            surge *= 0.35
        return float(surge), wz


class SafetyFilter:
    """前向声呐安全滤波：前方余量不足时减速/急停。"""

    def __init__(self, min_clearance: float = 0.80,
                 hard_clearance: float = 0.50,
                 half_fov_deg: float = 55.0) -> None:
        self.min_clearance = float(min_clearance)
        self.hard_clearance = float(hard_clearance)
        self.half_fov = math.radians(float(half_fov_deg))

    def frontal_clearance(self, scan: Sequence[float],
                          beam_angles: Sequence[float]) -> float:
        """Minimum forward sonar range inside the safety sector (m)."""
        ranges = np.asarray(scan, dtype=np.float64).ravel()
        beams = np.asarray(beam_angles, dtype=np.float64).ravel()
        n = min(ranges.size, beams.size)
        if n == 0:
            return C.SONAR_RANGE_MAX
        front = np.abs(beams[:n]) <= self.half_fov
        valid = (np.isfinite(ranges[:n]) & (ranges[:n] < C.SONAR_RANGE_MAX * 0.995)
                 & (ranges[:n] > C.SONAR_RANGE_MIN + 0.02))
        sel = front & valid
        if not np.any(sel):
            return C.SONAR_RANGE_MAX
        return float(np.min(ranges[:n][sel]))

    def apply(self, cmd: Sequence[float], scan: Sequence[float],
              beam_angles: Sequence[float]) -> np.ndarray:
        """Return ``cmd`` with surge reduced/zeroed for the frontal clearance."""
        out = np.array(cmd, dtype=np.float64, copy=True)
        clearance = self.frontal_clearance(scan, beam_angles)
        if clearance < self.hard_clearance:
            out[0] = 0.0
        elif clearance < self.min_clearance:
            span = self.min_clearance - self.hard_clearance
            scale = max(0.0, (clearance - self.hard_clearance) / span)
            out[0] = max(0.0, out[0]) * (0.10 + 0.90 * scale)
        return out


class MissionController:
    """任务级控制器：A* 全局规划 + 纯跟踪 + 定深 + 安全滤波 + 重规划。

    Parameters
    ----------
    scene_or_map:
        :class:`scenes.Scene`（使用其 ``coarse_map``）或布尔占有栅格数组。
    resolution:
        规划栅格分辨率（m）。
    replan_interval:
        周期重规划间隔（s）。
    """

    def __init__(self, scene_or_map, resolution: float = C.PLAN_RESOLUTION,
                 replan_interval: float = 1.0) -> None:
        if hasattr(scene_or_map, "coarse_map"):
            grid = np.asarray(scene_or_map.coarse_map(resolution), dtype=bool)
        else:
            grid = np.asarray(scene_or_map, dtype=bool)
        self.resolution = float(resolution)
        # 多级膨胀半径：优先保持更大的安全余量，窄通道场景自动回退
        self._planners = [
            GridPlanner(grid, resolution, radius / self.resolution)
            for radius in (0.65, 0.55, 0.45)
        ]
        self.planner = self._planners[0]
        self.pursuit = PurePursuit()
        self.depth = DepthController()
        self.safety = SafetyFilter()
        self.fallback = GeometricPlanner()
        self.replan_interval = float(replan_interval)
        self.path: List[Tuple[float, float]] = []
        self.replans = 0
        self._goal: Optional[Tuple[float, float]] = None
        self._timer = 0.0
        self._blocked_time = 0.0

    # ---------------------------------------------------------------- helpers
    def _replan(self, pose: Sequence[float], goal_xy: Tuple[float, float]) -> None:
        path = None
        for planner in self._planners:
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
                      goal_xy[1] - self._goal[1]) > 0.25:
            return True
        if self._timer <= 0.0:
            return True
        if self._blocked_time > 1.0:
            return True
        # existing path invalidated by a newly observed obstacle
        for x, y in self.path:
            if not self.planner.is_free(x, y):
                return True
        return False

    # ----------------------------------------------------------------- update
    def update(self, pose: Sequence[float], goal: Sequence[float],
               scan: Sequence[float], beam_angles: Sequence[float],
               dt: float) -> np.ndarray:
        """Compute one control command ``[surge, 0, heave, yaw_rate]``."""
        pose = np.asarray(pose, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        self._timer -= float(dt)
        pose_xy = (float(pose[0]), float(pose[1]))
        goal_xy = (float(goal[0]), float(goal[1]))
        dist_goal = math.hypot(goal_xy[0] - pose_xy[0], goal_xy[1] - pose_xy[1])
        if self._needs_replan(pose_xy, goal_xy):
            self._replan(pose, goal_xy)

        if dist_goal <= 0.45:
            surge, wz = 0.0, 0.0
        elif self.path:
            surge, wz = self.pursuit.compute([pose[0], pose[1], pose[3]], self.path)
        else:
            surge, wz = self.fallback.compute(pose, goal_xy, scan, beam_angles)

        heave = self.depth.update(float(pose[2]), float(goal[2]), dt)
        cmd = np.array([surge, 0.0, heave, wz], dtype=np.float64)
        raw_surge = cmd[0]
        cmd = self.safety.apply(cmd, scan, beam_angles)
        if raw_surge > 0.15 and cmd[0] < 0.05:
            self._blocked_time += float(dt)
        else:
            self._blocked_time = 0.0
        return cmd
