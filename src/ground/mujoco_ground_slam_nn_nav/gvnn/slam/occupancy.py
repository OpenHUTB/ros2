"""2-D log-odds occupancy mapping."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from ..config import LIDAR_RANGE_MAX, MAP_RESOLUTION
from ..geometry import bresenham, transform_points

FREE = -0.42
OCC = 0.88
CLAMP = 4.0


class OccupancyMap:
    """Log-odds occupancy grid in the map frame (origin at world (0, 0))."""

    def __init__(self, width: float, height: float, resolution: float = MAP_RESOLUTION):
        self.width = float(width)
        self.height = float(height)
        self.resolution = float(resolution)
        self.w = int(round(width / resolution))
        self.h = int(round(height / resolution))
        self.log_odds = np.zeros((self.h, self.w), dtype=np.float32)
        self._score: Optional[np.ndarray] = None
        self.updates = 0

    def reset(self) -> None:
        self.log_odds.fill(0.0)
        self._score = None
        self.updates = 0

    # ------------------------------------------------------------------ update
    def update(self, pose: Sequence[float], ranges: Sequence[float],
               angle_min: float = 0.0, angle_increment: float = 2.0 * math.pi / 360.0,
               max_range: float = LIDAR_RANGE_MAX) -> None:
        """Vectorised inverse-sensor-model update.

        All beams are rasterised in one batch with ``numpy.bincount``; this is
        ~20x faster than a Python Bresenham loop and keeps 10 Hz mapping
        comfortably within the CPU budget of the VM.
        """
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        ox = int(x / self.resolution)
        oy = int(y / self.resolution)
        if not (0 <= ox < self.w and 0 <= oy < self.h):
            return
        r = np.asarray(ranges, dtype=np.float64)
        if r.size == 0:
            return
        r = np.clip(r, 0.05, max_range)
        angles = yaw + angle_min + np.arange(r.size) * angle_increment
        dx = r * np.cos(angles)
        dy = r * np.sin(angles)
        steps = np.maximum(1, np.ceil(r / self.resolution).astype(np.int32))
        max_steps = int(max_range / self.resolution) + 2
        s = np.arange(max_steps, dtype=np.int32)[None, :]
        valid = s < steps[:, None]
        t = (s + 1.0) / steps[:, None]
        wx = x + dx[:, None] * t
        wy = y + dy[:, None] * t
        ix = (wx / self.resolution).astype(np.int32)
        iy = (wy / self.resolution).astype(np.int32)
        inside = valid & (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        idx = np.where(inside, iy * self.w + ix, 0)

        hit = r < max_range - 0.08
        # Free space excludes the last cell of each hit ray.
        free_mask = inside & (s < (steps - 1)[:, None])
        free_counts = np.bincount(idx[free_mask], minlength=self.h * self.w)
        # Endpoints of hit rays become occupied.
        ex = np.clip((x + dx) / self.resolution, 0, self.w - 1).astype(np.int32)
        ey = np.clip((y + dy) / self.resolution, 0, self.h - 1).astype(np.int32)
        occ_idx = ey[hit] * self.w + ex[hit]
        occ_counts = np.bincount(occ_idx, minlength=self.h * self.w)
        delta = free_counts.astype(np.float32) * FREE + occ_counts.astype(np.float32) * OCC
        np.clip(self.log_odds + delta.reshape(self.h, self.w), -CLAMP, CLAMP,
                out=self.log_odds)
        self.updates += 1
        self._score = None

    # ------------------------------------------------------------------ query
    def probability(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def score_field(self) -> np.ndarray:
        """Blurred occupancy probability used by correlative scan matching."""
        if self._score is None:
            p = self.probability()
            pad = np.pad(p, 1, mode="edge")
            out = p.copy()
            for oy in (-1, 0, 1):
                for ox in (-1, 0, 1):
                    out = np.maximum(out, pad[1 + oy:1 + oy + self.h, 1 + ox:1 + ox + self.w])
            self._score = out.astype(np.float32)
        return self._score

    def data(self, threshold: float = 0.35) -> np.ndarray:
        """ROS OccupancyGrid data: -1 unknown, 0 free, 100 occupied."""
        out = np.full((self.h, self.w), -1, dtype=np.int8)
        out[self.log_odds > threshold] = 100
        out[self.log_odds < -threshold] = 0
        return out

    def known_mask(self, threshold: float = 0.35) -> np.ndarray:
        return np.abs(self.log_odds) > threshold

    def occupied_mask(self, threshold: float = 0.35) -> np.ndarray:
        return self.log_odds > threshold

    def unknown_mask(self, threshold: float = 0.35) -> np.ndarray:
        return np.abs(self.log_odds) <= threshold

    def coverage(self) -> float:
        """Fraction of cells that have been observed (not unknown)."""
        return float(self.known_mask().mean())

    def occupied_iou(self, truth_occ: np.ndarray) -> float:
        """IoU of occupied cells inside the observed region."""
        known = self.known_mask()
        est = self.occupied_mask() & known
        gt = np.asarray(truth_occ, dtype=bool) & known
        union = np.count_nonzero(est | gt)
        if union == 0:
            return 0.0
        return float(np.count_nonzero(est & gt) / union)

    # ------------------------------------------------------------------ points
    def points_in_frame(self, pose: Sequence[float], ranges: Sequence[float],
                        angle_min: float = 0.0,
                        angle_increment: float = 2.0 * math.pi / 360.0) -> np.ndarray:
        a = float(pose[2]) + angle_min + np.arange(len(ranges)) * angle_increment
        r = np.asarray(ranges, dtype=np.float64)
        pts = np.column_stack([r * np.cos(a), r * np.sin(a)])
        return transform_points(pts, pose)


class ScanMatcher:
    """Coarse-to-fine correlative scan matcher against an occupancy map."""

    def __init__(self, grid: OccupancyMap, max_range: float = LIDAR_RANGE_MAX,
                 stride: int = 3, prior_translation: float = 1.0,
                 prior_rotation: float = 0.8, search_xy: float = 0.36,
                 search_yaw_deg: float = 6.0):
        self.grid = grid
        self.max_range = float(max_range)
        self.stride = int(stride)
        self.prior_translation = float(prior_translation)
        self.prior_rotation = float(prior_rotation)
        self.search_xy = float(search_xy)
        self.search_yaw_deg = float(search_yaw_deg)
        self.last_score = 0.0
        self.last_delta = np.zeros(3)
        self._angles = None

    def _points(self, ranges: Sequence[float], angle_min: float,
                angle_increment: float) -> np.ndarray:
        r = np.asarray(ranges, dtype=np.float64)
        valid = np.isfinite(r) & (r > 0.12) & (r < self.max_range - 0.12)
        idx = np.arange(len(r))[valid]
        idx = idx[::self.stride]
        a = angle_min + idx * angle_increment
        return np.column_stack([r[idx] * np.cos(a), r[idx] * np.sin(a)])

    def _score_pose(self, pts: np.ndarray, pose: Sequence[float],
                    field: np.ndarray) -> float:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(yaw), math.sin(yaw)
        wx = c * pts[:, 0] - s * pts[:, 1] + x
        wy = s * pts[:, 0] + c * pts[:, 1] + y
        ix = np.clip((wx / self.grid.resolution).astype(np.int32), 0, self.grid.w - 1)
        iy = np.clip((wy / self.grid.resolution).astype(np.int32), 0, self.grid.h - 1)
        return float(field[iy, ix].mean())

    def _batch_search(self, pts: np.ndarray, base: Sequence[float],
                      dxs: np.ndarray, dys: np.ndarray,
                      dyaws: np.ndarray, field: np.ndarray):
        """Evaluate all candidate poses in one vectorised NumPy operation."""
        gx, gy, ga = np.meshgrid(dxs, dys, dyaws, indexing="ij")
        dx = gx.ravel()
        dy = gy.ravel()
        da = ga.ravel()
        c = np.cos(base[2] + da)
        s = np.sin(base[2] + da)
        wx = c[:, None] * pts[None, :, 0] - s[:, None] * pts[None, :, 1] + (base[0] + dx)[:, None]
        wy = s[:, None] * pts[None, :, 0] + c[:, None] * pts[None, :, 1] + (base[1] + dy)[:, None]
        ix = np.clip((wx / self.grid.resolution).astype(np.int32), 0, self.grid.w - 1)
        iy = np.clip((wy / self.grid.resolution).astype(np.int32), 0, self.grid.h - 1)
        score = field[iy, ix].mean(axis=1)
        score -= self.prior_translation * (dx * dx + dy * dy)
        score -= self.prior_rotation * (da * da)
        best = int(np.argmax(score))
        return (float(base[0] + dx[best]), float(base[1] + dy[best]),
                float(base[2] + da[best])), float(score[best])

    def match(self, predicted: Sequence[float], ranges: Sequence[float],
              angle_min: float = 0.0,
              angle_increment: float = 2.0 * math.pi / 360.0) -> Tuple[np.ndarray, float]:
        field = self.grid.score_field()
        pts = self._points(ranges, angle_min, angle_increment)
        if len(pts) < 20 or self.grid.updates < 2:
            self.last_delta[:] = 0.0
            self.last_score = 0.0
            return np.array(predicted, dtype=np.float64), 0.0

        px, py, pyaw = float(predicted[0]), float(predicted[1]), float(predicted[2])
        best, best_score = self._batch_search(
            pts, (px, py, pyaw),
            np.arange(-self.search_xy, self.search_xy + 1e-9, 0.12),
            np.arange(-self.search_xy, self.search_xy + 1e-9, 0.12),
            np.deg2rad(np.arange(-self.search_yaw_deg, self.search_yaw_deg + 1e-9, 2.0)),
            field,
        )
        best, best_score = self._batch_search(
            pts, best,
            np.arange(-0.11, 0.111, 0.022),
            np.arange(-0.11, 0.111, 0.022),
            np.deg2rad(np.arange(-1.4, 1.41, 0.4)),
            field,
        )
        self.last_score = best_score
        self.last_delta = np.array(best) - np.array([px, py, pyaw], dtype=np.float64)
        # Reject a match that is not better than the odometry prior.
        if best_score < 0.54:
            self.last_delta[:] = 0.0
            return np.array(predicted, dtype=np.float64), best_score
        return np.array(best, dtype=np.float64), best_score
