"""Pure-pursuit path tracking with a curvature-aware speed policy."""

from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

import numpy as np

from ..config import MAX_OMEGA, MAX_SPEED
from ..geometry import wrap_angle


class PurePursuit:
    def __init__(self, lookahead: float = 0.95, max_speed: float = 1.0,
                 wheelbase: float = 0.56):
        self.lookahead = float(lookahead)
        self.max_speed = float(max_speed)
        self.wheelbase = float(wheelbase)

    @staticmethod
    def _project(path: np.ndarray, x: float, y: float) -> Tuple[float, float, int, float]:
        """Project onto the polyline.

        Returns (arc length s, signed lateral error, segment index, segment
        fraction t).  Signed error is positive when the robot is left of the
        path direction.
        """
        best = (0.0, 0.0, 0, 0.0)
        best_d2 = float("inf")
        s = 0.0
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            vx, vy = x1 - x0, y1 - y0
            seg2 = vx * vx + vy * vy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - x0) * vx + (y - y0) * vy) / seg2))
            px, py = x0 + t * vx, y0 + t * vy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                seg = math.sqrt(seg2)
                cross = ((x - x0) * vy - (y - y0) * vx) / seg
                best_d2 = d2
                best = (s + t * seg, cross, i, t)
            s += math.sqrt(seg2)
        return best

    @staticmethod
    def _point_at_s(path: np.ndarray, s: float) -> Tuple[float, float]:
        if s <= 0:
            return float(path[0, 0]), float(path[0, 1])
        acc = 0.0
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                continue
            if acc + seg >= s:
                t = (s - acc) / seg
                return float(x0 + t * (x1 - x0)), float(y0 + t * (y1 - y0))
            acc += seg
        return float(path[-1, 0]), float(path[-1, 1])

    def control(self, pose: Sequence[float], path: Sequence[Sequence[float]],
                front_clearance: float = 9.0, goal_tolerance: float = 0.55,
                speed_scale: float = 1.0) -> Dict[str, float]:
        if path is None or len(path) == 0:
            return {"v": 0.0, "w": 0.0, "curvature": 0.0, "heading_error": 0.0,
                    "cross_track": 0.0, "lookahead": self.lookahead, "distance": 0.0}
        pts = np.asarray(path, dtype=np.float64)
        if len(pts) == 1:
            pts = np.vstack([pts, pts])
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        s, cross, ci, ti = self._project(pts, x, y)
        # Arc-length lookahead: never behind the robot, even with sparse waypoints.
        target = self._point_at_s(pts, s + self.lookahead)
        dx, dy = target[0] - x, target[1] - y
        L = max(0.25, math.hypot(dx, dy))
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        curvature = 2.0 * math.sin(alpha) / L

        v = self.max_speed * speed_scale
        v *= max(0.40, 1.0 - 0.50 * min(1.0, abs(curvature) * 1.8))
        if front_clearance < 1.6:
            v *= max(0.18, (front_clearance - 0.35) / 1.25)
        dxg = pts[-1, 0] - x
        dyg = pts[-1, 1] - y
        dist_goal = math.hypot(dxg, dyg)
        v *= max(0.25, min(1.0, dist_goal / 1.6))
        v = float(np.clip(v, 0.0, MAX_SPEED))
        if dist_goal < goal_tolerance:
            return {"v": 0.0, "w": 0.0, "curvature": curvature,
                    "heading_error": alpha, "cross_track": cross,
                    "lookahead": L, "distance": dist_goal}
        if abs(alpha) > 0.85:
            v = min(v, 0.12)
            w = 0.85 * (1.0 if alpha > 0 else -1.0)
        else:
            w = v * curvature
        w = float(np.clip(w, -MAX_OMEGA, MAX_OMEGA))
        return {"v": v, "w": w, "curvature": curvature, "heading_error": alpha,
                "cross_track": cross, "lookahead": L, "distance": dist_goal}
