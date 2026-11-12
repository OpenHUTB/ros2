"""Occupancy-grid mapping and scan-matching SLAM front end for the humanoid.

Pose convention: ``(x, y, z, yaw)`` -- metres in the world frame and radians for
the heading.  The robot walks on a single floor, so ``z`` is always 0 and is
passed through untouched (the same slot carried depth / altitude in the other
vehicles of this template family).

The indoor map is small (hall 30 x 12 m = 375 x 150 cells at
:data:`config.MAP_RESOLUTION`, 0.08 m), so the front end keeps the vectorised
log-odds ray casting of the shared template and adds coarse-to-fine
correlation scan matching for drift correction:

* :class:`OccupancyGridMap` -- log-odds occupancy grid with vectorised ray
  casting and incremental clipping (only touched cells are clipped);
* :class:`SlamFrontend` -- odometry + lidar coarse-to-fine correlation scan
  matching localisation front end.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle

__all__ = ["OccupancyGridMap", "SlamFrontend", "lidar_beam_angles"]

# ------------------------------------------------------------- sensor limits
_RANGE_MAX = float(C.LIDAR_RANGE_MAX)
_RANGE_MIN = float(C.LIDAR_RANGE_MIN)
_BEAMS = int(C.LIDAR_BEAMS)
_FOV = float(C.LIDAR_FOV)
_TWO_PI = 2.0 * math.pi

# ---------------------------------------------------------------- log-odds
_LOG_ODDS_MIN = -3.5
_LOG_ODDS_MAX = 3.5
_L_FREE = -0.55
_L_OCC = 0.90

# ----------------------------------------------- scan-matching search setup
_COARSE_XY = 0.24          # m, coarse search half span
_COARSE_XY_STEP = 0.08
_COARSE_YAW = 0.08         # rad
_COARSE_YAW_STEP = 0.04
_FINE_XY = 0.030           # m, fine search half span around the coarse optimum
_FINE_XY_STEP = 0.020
_FINE_YAW = 0.020          # rad
_FINE_YAW_STEP = 0.010
_MAX_CORRECTION_XY = 0.15  # trust region: allowed correction per update (m)
_MAX_CORRECTION_YAW = 0.05  # allowed correction per update (rad)
_MIN_HIT_EVIDENCE = 20.0   # occupied log-odds mass needed before matching
_MIN_VALID_BEAMS = 8
_MIN_SCORE_GAIN = 3.0      # reject matches that barely beat the odometry prior


def lidar_beam_angles(fov: float = _FOV, beams: int = _BEAMS) -> np.ndarray:
    """Beam angles (rad, relative to the heading) matching :class:`sim.HumanoidSim`."""
    fov = float(fov)
    beams = int(beams)
    if beams <= 0:
        return np.zeros(0, dtype=np.float64)
    if fov >= _TWO_PI - 1e-6:
        return np.linspace(-math.pi, math.pi, beams, endpoint=False,
                           dtype=np.float64)
    return np.linspace(-0.5 * fov, 0.5 * fov, beams, dtype=np.float64)


def _offsets(xy_span: float, xy_step: float,
             yaw_span: float, yaw_step: float) -> np.ndarray:
    """Build a grid of ``(dx, dy, dyaw)`` search offsets."""
    xs = np.arange(-xy_span, xy_span + 1e-9, xy_step, dtype=np.float64)
    ys = xs
    yaws = np.arange(-yaw_span, yaw_span + 1e-9, yaw_step, dtype=np.float64)
    gx, gy, gyaw = np.meshgrid(xs, ys, yaws, indexing="ij")
    return np.column_stack([gx.ravel(), gy.ravel(), gyaw.ravel()])


def _score_candidates(log_odds: np.ndarray, resolution: float,
                      x0: float, y0: float, yaw0: float,
                      ranges: np.ndarray, beams: np.ndarray,
                      offsets: np.ndarray) -> np.ndarray:
    """Score candidate poses by the occupied evidence under the beam endpoints."""
    res = float(resolution)
    h, w = log_odds.shape
    dx = offsets[:, 0][:, None]
    dy = offsets[:, 1][:, None]
    dyaw = offsets[:, 2][:, None]
    ang = yaw0 + dyaw + beams[None, :]
    ex = x0 + dx + ranges[None, :] * np.cos(ang)
    ey = y0 + dy + ranges[None, :] * np.sin(ang)
    cols = np.floor(ex / res).astype(np.int64)
    rows = np.floor(ey / res).astype(np.int64)
    inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    cols = np.clip(cols, 0, w - 1)
    rows = np.clip(rows, 0, h - 1)
    vals = log_odds[rows, cols]
    score = np.sum(np.where(inside, np.maximum(vals, 0.0), 0.0), axis=1)
    # offset prior: sigma_xy = 0.30 m, sigma_yaw = 0.10 rad
    score -= 2.0 * (offsets[:, 0] ** 2 + offsets[:, 1] ** 2) / (2.0 * 0.30 ** 2)
    score -= 1.0 * offsets[:, 2] ** 2 / (2.0 * 0.10 ** 2)
    return score


class OccupancyGridMap:
    """2-D log-odds occupancy grid covering the whole indoor floor plan.

    Parameters
    ----------
    width_m, height_m:
        World extent of the map (m); the origin is ``(0, 0)``.
    resolution:
        Cell size in metres, default :data:`config.MAP_RESOLUTION`.
    """

    def __init__(self, width_m: float, height_m: float,
                 resolution: float = C.MAP_RESOLUTION) -> None:
        self.resolution = float(resolution)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.width = max(1, int(round(self.width_m / self.resolution)))
        self.height = max(1, int(round(self.height_m / self.resolution)))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    # ------------------------------------------------------------ conversion
    def from_world(self, x: float, y: float) -> Tuple[int, int]:
        """World ``(x, y)`` -> grid ``(col, row)`` (clipped to the map)."""
        c = int(math.floor(float(x) / self.resolution))
        r = int(math.floor(float(y) / self.resolution))
        return min(max(c, 0), self.width - 1), min(max(r, 0), self.height - 1)

    def to_world(self, c: int, r: int) -> Tuple[float, float]:
        """Grid ``(col, row)`` -> world cell centre ``(x, y)``."""
        return (float(c) + 0.5) * self.resolution, (float(r) + 0.5) * self.resolution

    # ---------------------------------------------------------------- update
    def update(self, pose_xy: Sequence[float], ranges: Sequence[float],
               beam_angles_rel: Sequence[float],
               max_range: Optional[float] = None) -> None:
        """Fuse one lidar scan into the grid with vectorised log-odds ray casting.

        ``pose_xy`` is ``(x, y)`` with an optional yaw (3rd element, or the 4th
        if the full ``(x, y, z, yaw)`` pose is passed).  ``beam_angles_rel`` are
        beam angles relative to the heading.
        """
        pose = np.asarray(pose_xy, dtype=np.float64).ravel()
        if pose.size < 2:
            return
        x, y = float(pose[0]), float(pose[1])
        if pose.size >= 4:
            yaw = float(pose[3])
        elif pose.size >= 3:
            yaw = float(pose[2])
        else:
            yaw = 0.0
        ranges = np.asarray(ranges, dtype=np.float64).ravel()
        beams = np.asarray(beam_angles_rel, dtype=np.float64).ravel()
        n = min(ranges.size, beams.size)
        if n == 0:
            return
        r_max = float(max_range) if max_range is not None else _RANGE_MAX
        ranges, beams = ranges[:n], beams[:n]
        valid = (np.isfinite(ranges) & (ranges >= _RANGE_MIN + 0.02)
                 & (ranges <= r_max * 0.995))
        if not np.any(valid):
            return
        ranges = ranges[valid]
        angles = yaw + beams[valid]

        res = self.resolution
        w, h = self.width, self.height
        flat = self.log_odds.reshape(-1)
        step = res * 0.5

        # ---- free space: sample every half cell along each ray (vectorised)
        counts = np.floor(np.maximum(ranges - res, 0.0) / step).astype(np.int64)
        total = int(counts.sum())
        touched = []
        if total > 0:
            starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
            beam_idx = np.repeat(np.arange(ranges.size), counts)
            offs = np.arange(total) - np.repeat(starts, counts) + 1
            t = offs.astype(np.float64) * step
            px = x + np.cos(angles[beam_idx]) * t
            py = y + np.sin(angles[beam_idx]) * t
            cols = np.floor(px / res).astype(np.int64)
            rows = np.floor(py / res).astype(np.int64)
            ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
            if np.any(ok):
                packed = np.unique(rows[ok] * w + cols[ok])
                flat[packed] += _L_FREE
                touched.append(packed)

        # ---- occupied endpoints (duplicates accumulate)
        hx = x + np.cos(angles) * ranges
        hy = y + np.sin(angles) * ranges
        hc = np.floor(hx / res).astype(np.int64)
        hr = np.floor(hy / res).astype(np.int64)
        ok = (hc >= 0) & (hc < w) & (hr >= 0) & (hr < h)
        if np.any(ok):
            hit_idx = hr[ok] * w + hc[ok]
            np.add.at(flat, hit_idx, _L_OCC)
            touched.append(hit_idx)

        if touched:
            idx = np.unique(np.concatenate(touched))
            flat[idx] = np.clip(flat[idx], _LOG_ODDS_MIN, _LOG_ODDS_MAX)

    # ------------------------------------------------------------- read-out
    @property
    def probability(self) -> np.ndarray:
        """Occupancy probability in ``[0, 1]``, ``float32`` of shape ``(H, W)``."""
        return (1.0 / (1.0 + np.exp(-self.log_odds))).astype(np.float32)

    def occupied(self, threshold: float = 0.65) -> np.ndarray:
        """Boolean map of cells considered occupied."""
        return self.probability > float(threshold)

    def free_mask(self, threshold: float = 0.35) -> np.ndarray:
        """Boolean map of cells considered free."""
        return self.probability < float(threshold)

    def known_mask(self, threshold: float = 0.25) -> np.ndarray:
        """Cells with enough evidence to be considered "seen" (free or occupied)."""
        return np.abs(self.log_odds) >= float(threshold)

    # ------------------------------------------------------------------ io
    def save(self, path: str) -> None:
        """Save the map to an ``.npz`` file."""
        np.savez(path, log_odds=self.log_odds,
                 resolution=np.float64(self.resolution),
                 width_m=np.float64(self.width_m),
                 height_m=np.float64(self.height_m))

    def load(self, path: str) -> None:
        """Load a map previously written by :meth:`save`."""
        with np.load(path) as data:
            self.log_odds = np.asarray(data["log_odds"], dtype=np.float32)
            self.resolution = float(data["resolution"])
            self.width_m = float(data["width_m"])
            self.height_m = float(data["height_m"])
        self.height, self.width = self.log_odds.shape


class SlamFrontend:
    """Odometry + lidar 2-D SLAM front end (scan matching + occupancy mapping).

    ``scene`` is only used to size the grid (world extent); the ground-truth
    floor plan is never read.  Pose convention is ``[x, y, z, yaw]``; ``z`` is
    copied through unchanged (the robot is planar).
    """

    def __init__(self, scene, use_scan_matching: bool = True,
                 resolution: float = C.MAP_RESOLUTION) -> None:
        self.scene = scene
        self.resolution = float(resolution)
        width = float(getattr(scene, "width", _RANGE_MAX * 2.0))
        height = float(getattr(scene, "height", _RANGE_MAX * 2.0))
        self.map = OccupancyGridMap(width, height, self.resolution)
        self.use_scan_matching = bool(use_scan_matching)
        self.trajectory: List[np.ndarray] = []
        self.corrections: List[np.ndarray] = []
        self._beam_angles = lidar_beam_angles()
        self._coarse = _offsets(_COARSE_XY, _COARSE_XY_STEP,
                                _COARSE_YAW, _COARSE_YAW_STEP)
        self._fine = _offsets(_FINE_XY, _FINE_XY_STEP,
                              _FINE_YAW, _FINE_YAW_STEP)
        self.updates = 0

    # --------------------------------------------------------------- public
    def update(self, odom_pose: Sequence[float],
               ranges: Sequence[float]) -> np.ndarray:
        """Fuse one scan; return the corrected pose ``[x, y, z, yaw]``."""
        odom = np.asarray(odom_pose, dtype=np.float64).ravel()
        if odom.size < 4:
            odom = np.array([odom[0] if odom.size > 0 else 0.0,
                             odom[1] if odom.size > 1 else 0.0,
                             odom[2] if odom.size > 2 else 0.0,
                             0.0], dtype=np.float64)
        if self.use_scan_matching:
            pose = self._match(odom, ranges)
        else:
            pose = odom.copy()
        # z slot is passed through untouched (planar robot: always 0)
        self.map.update(np.array([pose[0], pose[1], pose[3]]),
                        ranges, self._beam_angles)
        self.trajectory.append(np.array(pose, dtype=np.float64, copy=True))
        self.updates += 1
        return pose

    def ate_rmse(self, truth_poses: Sequence[Sequence[float]]) -> float:
        """RMSE of the estimated trajectory against ``truth_poses`` in x-y (m)."""
        n = min(len(self.trajectory), len(truth_poses))
        if n == 0:
            return float("nan")
        est = np.asarray(self.trajectory[:n], dtype=np.float64)[:, :2]
        gt = np.asarray(truth_poses[:n], dtype=np.float64)[:, :2]
        err = np.linalg.norm(est - gt, axis=1)
        return float(np.sqrt(np.mean(err * err)))

    def final_xy_error(self, truth_poses: Sequence[Sequence[float]]) -> float:
        """Horizontal error of the last estimate against the last truth pose."""
        n = min(len(self.trajectory), len(truth_poses))
        if n == 0:
            return float("nan")
        est = np.asarray(self.trajectory[n - 1], dtype=np.float64)
        gt = np.asarray(truth_poses[n - 1], dtype=np.float64)
        return float(math.hypot(est[0] - gt[0], est[1] - gt[1]))

    # ------------------------------------------------------------- matching
    def _match(self, odom: np.ndarray, ranges: Sequence[float]) -> np.ndarray:
        ranges = np.asarray(ranges, dtype=np.float64).ravel()
        n = min(ranges.size, self._beam_angles.size)
        ranges = ranges[:n]
        beams = self._beam_angles[:n]
        valid = (np.isfinite(ranges) & (ranges >= _RANGE_MIN + 0.05)
                 & (ranges <= _RANGE_MAX * 0.995))
        if int(valid.sum()) < _MIN_VALID_BEAMS:
            return odom.copy()
        log_odds = self.map.log_odds
        if float(np.sum(np.maximum(log_odds, 0.0))) < _MIN_HIT_EVIDENCE:
            return odom.copy()
        r = ranges[valid]
        b = beams[valid]
        x0, y0, yaw0 = float(odom[0]), float(odom[1]), float(odom[3])

        coarse_scores = _score_candidates(log_odds, self.map.resolution,
                                          x0, y0, yaw0, r, b, self._coarse)
        best_i = int(np.argmax(coarse_scores))
        best_off = self._coarse[best_i].copy()
        best_score = float(coarse_scores[best_i])

        fine = self._fine + best_off[None, :]
        fine_scores = _score_candidates(log_odds, self.map.resolution,
                                        x0, y0, yaw0, r, b, fine)
        fine_i = int(np.argmax(fine_scores))
        if float(fine_scores[fine_i]) > best_score:
            best_off = fine[fine_i].copy()
            best_score = float(fine_scores[fine_i])

        odom_score = float(_score_candidates(
            log_odds, self.map.resolution, x0, y0, yaw0, r, b,
            np.zeros((1, 3), dtype=np.float64))[0])
        if best_score <= odom_score + _MIN_SCORE_GAIN:
            return odom.copy()

        dx = float(np.clip(best_off[0], -_MAX_CORRECTION_XY, _MAX_CORRECTION_XY))
        dy = float(np.clip(best_off[1], -_MAX_CORRECTION_XY, _MAX_CORRECTION_XY))
        dyaw = float(np.clip(best_off[2], -_MAX_CORRECTION_YAW,
                             _MAX_CORRECTION_YAW))
        self.corrections.append(np.array([dx, dy, dyaw], dtype=np.float64))
        return np.array([x0 + dx, y0 + dy, float(odom[2]),
                         wrap_angle(yaw0 + dyaw)], dtype=np.float64)
