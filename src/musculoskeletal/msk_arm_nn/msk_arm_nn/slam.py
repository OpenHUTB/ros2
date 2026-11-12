"""Workspace occupancy mapping from the arm-plane depth camera (MSK-Arm).

This replaces the mobile-robot SLAM of the shared template with what a fixed
musculoskeletal arm actually needs: a **workspace occupancy map** built from the
forward depth camera.

The depth camera of :class:`sim.MskArmSim` is a 32 x 32 image of a 120 deg fan
around the arm base (distance encoded as the height of the illuminated column:
``prox = h / H``, ``dist = CAM_RANGE * (1 - prox)``).  The front end decodes the
image back to per-column ranges, then rasterises the visible workspace: cells
closer to the origin than the measured range are carved free, cells around the
measured range are marked occupied.  Log-odds are accumulated so repeated scans
produce a stable map.

* :class:`OccupancyGridMap` -- log-odds workspace grid (0.01 m cells);
* :class:`SlamFrontend` -- depth-camera -> workspace map front end, with
  :meth:`SlamFrontend.occupancy` / :attr:`SlamFrontend.probability` read-out.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import wrap_angle

__all__ = ["OccupancyGridMap", "SlamFrontend", "decode_depth_ranges",
           "depth_beam_angles"]

_LOG_ODDS_MIN = -2.5
_LOG_ODDS_MAX = 3.0
_L_FREE = -0.40
_L_OCC = 0.85

#: the simulator points the depth camera slightly up towards the target area
CAM_BASE_ANGLE = 0.6


def depth_beam_angles(n_beams: Optional[int] = None,
                      fov: float = C.CAM_FOV,
                      base_angle: float = CAM_BASE_ANGLE) -> np.ndarray:
    """Absolute beam angles (rad) of the arm-plane depth camera.

    Mirrors the camera model of :meth:`sim.MskArmSim.depth_camera`.
    """
    n = int(n_beams if n_beams is not None else C.CAM_WIDTH)
    half = 0.5 * float(fov)
    return (float(base_angle) - half
            + float(fov) * (np.arange(n, dtype=np.float64) + 0.5) / n)


def decode_depth_ranges(depth: Sequence[float],
                        cam_range: float = C.CAM_RANGE,
                        threshold: float = 0.5) -> np.ndarray:
    """Decode the simulator depth image into one range per image column.

    The simulator writes ``1.0`` in the bottom ``h = round(H * prox)`` rows of
    each column (plus a dark floor gradient), so the number of bright pixels is
    the proximity measure.  Columns with no bright pixel are "no return" and get
    the full camera range.
    """
    img = np.asarray(depth, dtype=np.float64)
    if img.ndim == 3:
        img = img[..., 0]
    if img.ndim != 2 or img.size == 0:
        return np.full(int(C.CAM_WIDTH), float(cam_range))
    h, w = img.shape
    if w != int(C.CAM_WIDTH):
        # resample the columns if a different camera width is passed
        cols = np.linspace(0, w - 1, int(C.CAM_WIDTH))
        img = img[:, np.round(cols).astype(np.int64)]
        w = img.shape[1]
    bright = img > float(threshold)
    counts = bright.sum(axis=0).astype(np.float64)
    prox = np.clip(counts / float(h), 0.0, 1.0)
    ranges = float(cam_range) * (1.0 - prox)
    return np.clip(ranges, 0.0, float(cam_range))


class OccupancyGridMap:
    """2-D log-odds occupancy grid over the arm workspace.

    Parameters
    ----------
    width_m, height_m:
        Extent of the grid (m).
    origin:
        World coordinate ``(x0, y0)`` of the lower-left cell corner.
    resolution:
        Cell size in metres, default :data:`config.MAP_RESOLUTION` (0.01 m).
    """

    def __init__(self, width_m: float, height_m: float,
                 origin: Tuple[float, float] = (C.WORK_X[0], C.WORK_Y[0]),
                 resolution: float = C.MAP_RESOLUTION) -> None:
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.width = max(1, int(round(self.width_m / self.resolution)))
        self.height = max(1, int(round(self.height_m / self.resolution)))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    # ------------------------------------------------------------ conversion
    def from_world(self, x: float, y: float) -> Tuple[int, int]:
        """World ``(x, y)`` -> grid ``(col, row)`` (clipped to the map)."""
        c = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        r = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        return min(max(c, 0), self.width - 1), min(max(r, 0), self.height - 1)

    def to_world(self, c: int, r: int) -> Tuple[float, float]:
        """Grid ``(col, row)`` -> world cell centre ``(x, y)``."""
        return (self.origin[0] + (float(c) + 0.5) * self.resolution,
                self.origin[1] + (float(r) + 0.5) * self.resolution)

    def cell_centers(self) -> Tuple[np.ndarray, np.ndarray]:
        """World ``(X, Y)`` coordinate arrays of all cell centres."""
        xs = self.origin[0] + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    # ---------------------------------------------------------------- update
    def integrate(self, free_mask: np.ndarray, occupied_mask: np.ndarray,
                  l_free: float = _L_FREE,
                  l_occ: float = _L_OCC) -> None:
        """Fuse boolean free/occupied cell masks into the log-odds grid."""
        free = np.asarray(free_mask, dtype=bool)
        occ = np.asarray(occupied_mask, dtype=bool)
        if free.shape != self.log_odds.shape or occ.shape != self.log_odds.shape:
            raise ValueError("masks must match the map shape")
        if np.any(occ):
            self.log_odds[occ] += float(l_occ)
        if np.any(free):
            self.log_odds[free] += float(l_free)
        np.clip(self.log_odds, _LOG_ODDS_MIN, _LOG_ODDS_MAX, out=self.log_odds)

    def update_rays(self, origin_xy: Sequence[float],
                    beam_angles: Sequence[float],
                    ranges: Sequence[float],
                    max_range: float = C.CAM_RANGE) -> None:
        """Generic vectorised ray-casting update (free along the ray, hit at end)."""
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        ang = np.asarray(beam_angles, dtype=np.float64).ravel()
        rng = np.asarray(ranges, dtype=np.float64).ravel()
        n = min(ang.size, rng.size)
        if n == 0:
            return
        ang, rng = ang[:n], rng[:n]
        valid = np.isfinite(rng) & (rng > 0.0)
        if not np.any(valid):
            return
        ang, rng = ang[valid], np.clip(rng[valid], 0.0, float(max_range))
        res = self.resolution
        w, h = self.width, self.height
        flat = self.log_odds.reshape(-1)
        step = res * 0.5
        counts = np.floor(np.maximum(rng - res, 0.0) / step).astype(np.int64)
        total = int(counts.sum())
        if total > 0:
            starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
            beam_idx = np.repeat(np.arange(rng.size), counts)
            offs = np.arange(total) - np.repeat(starts, counts) + 1
            t = offs.astype(np.float64) * step
            px = ox + np.cos(ang[beam_idx]) * t
            py = oy + np.sin(ang[beam_idx]) * t
            cols = np.floor((px - self.origin[0]) / res).astype(np.int64)
            rows = np.floor((py - self.origin[1]) / res).astype(np.int64)
            ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
            if np.any(ok):
                packed = np.unique(rows[ok] * w + cols[ok])
                flat[packed] += _L_FREE
        hx = ox + np.cos(ang) * rng
        hy = oy + np.sin(ang) * rng
        cols = np.floor((hx - self.origin[0]) / res).astype(np.int64)
        rows = np.floor((hy - self.origin[1]) / res).astype(np.int64)
        ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        if np.any(ok):
            np.add.at(flat, rows[ok] * w + cols[ok], _L_OCC)
        np.clip(self.log_odds, _LOG_ODDS_MIN, _LOG_ODDS_MAX, out=self.log_odds)

    # ------------------------------------------------------------- read-out
    @property
    def probability(self) -> np.ndarray:
        """Occupancy probability in ``[0, 1]``, ``float32`` of shape ``(H, W)``."""
        return (1.0 / (1.0 + np.exp(-self.log_odds))).astype(np.float32)

    def occupied(self, threshold: float = 0.60) -> np.ndarray:
        """Boolean map of cells considered occupied."""
        return self.probability > float(threshold)

    def free_mask(self, threshold: float = 0.40) -> np.ndarray:
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
                 height_m=np.float64(self.height_m),
                 origin=np.asarray(self.origin, dtype=np.float64))

    def load(self, path: str) -> None:
        """Load a map previously written by :meth:`save`."""
        with np.load(path) as data:
            self.log_odds = np.asarray(data["log_odds"], dtype=np.float32)
            self.resolution = float(data["resolution"])
            self.width_m = float(data["width_m"])
            self.height_m = float(data["height_m"])
            if "origin" in data:
                self.origin = (float(data["origin"][0]),
                               float(data["origin"][1]))
        self.height, self.width = self.log_odds.shape


class SlamFrontend:
    """Depth-camera -> workspace occupancy map front end.

    ``scene`` is only used for the workspace extent (the arm base is the
    origin); the ground-truth obstacles are never read by the mapping itself.
    """

    def __init__(self, scene, resolution: float = C.MAP_RESOLUTION,
                 use_truth_extent: bool = True) -> None:
        self.scene = scene
        self.resolution = float(resolution)
        if use_truth_extent:
            x0, x1 = C.WORK_X
            y0, y1 = C.WORK_Y
        else:
            x0 = y0 = -0.05
            x1, y1 = 0.64, 0.64
        self.origin = (x0, y0)
        self.map = OccupancyGridMap(x1 - x0, y1 - y0, origin=self.origin,
                                    resolution=self.resolution)
        self._angles = depth_beam_angles()
        self._n_beams = self._angles.size
        self._last_ranges: Optional[np.ndarray] = None
        self.updates = 0
        # pre-computed cell geometry for the visibility rasteriser
        self._X, self._Y = self.map.cell_centers()
        self._radius = np.hypot(self._X, self._Y)
        self._theta = np.arctan2(self._Y, self._X)

    # --------------------------------------------------------------- public
    def update(self, depth: Sequence[float]) -> np.ndarray:
        """Fuse one depth image; return the occupancy probability map."""
        ranges = decode_depth_ranges(depth)
        self._last_ranges = np.array(ranges, dtype=np.float64, copy=True)
        free, occupied = self._visible_masks(ranges)
        self.map.integrate(free, occupied)
        self.updates += 1
        return self.map.probability

    @property
    def probability(self) -> np.ndarray:
        return self.map.probability

    def occupancy(self, threshold: float = 0.60) -> np.ndarray:
        return self.map.occupied(threshold)

    def free_mask(self, threshold: float = 0.40) -> np.ndarray:
        return self.map.free_mask(threshold)

    def known_mask(self, threshold: float = 0.25) -> np.ndarray:
        return self.map.known_mask(threshold)

    def visible_mask(self) -> np.ndarray:
        """Cells inside the depth-camera fan (regardless of measurement)."""
        return self._in_fan()

    def observable_mask(self) -> np.ndarray:
        """Cells of the fan that are not hidden behind the first surface.

        Uses the ranges of the last scan: a cell is observable when it lies
        closer to the arm base than the measured range of its nearest beam (plus
        a small allowance for the 32-row depth quantisation).  Behind the shelf
        wall nothing is observable, and the coverage metric must not count it.
        """
        in_fan = self._in_fan()
        if self._last_ranges is None:
            return in_fan
        n = self._n_beams
        dtheta = float(C.CAM_FOV) / float(n)
        idx = np.floor((self._theta - float(self._angles[0])) / dtheta + 0.5)
        idx = np.clip(idx.astype(np.int64), 0, n - 1)
        nearest = np.clip(self._last_ranges, 0.0, float(C.CAM_RANGE))[idx]
        hit = nearest < float(C.CAM_RANGE) * 0.98
        return in_fan & ((~hit) | (self._radius <= nearest + 0.02))

    # ------------------------------------------------------------ internals
    def _in_fan(self) -> np.ndarray:
        n = self._n_beams
        half_step = 0.5 * (C.CAM_FOV / float(n))
        lo = float(self._angles[0]) - half_step
        hi = float(self._angles[-1]) + half_step
        return (self._theta >= lo) & (self._theta <= hi) \
            & (self._radius <= C.CAM_RANGE * 0.98)

    def _visible_masks(self, ranges: np.ndarray
                       ) -> Tuple[np.ndarray, np.ndarray]:
        """Rasterise the depth scan into free / occupied workspace cells.

        Each cell is associated with the nearest camera beam; it is free when it
        lies clearly in front of that beam's measured range and occupied when it
        sits at that range.  Beams without a return carve the whole visible ray.
        """
        in_fan = self._in_fan()
        n = self._n_beams
        dtheta = float(C.CAM_FOV) / float(n)
        idx = np.floor((self._theta - float(self._angles[0])) / dtheta + 0.5)
        idx = np.clip(idx.astype(np.int64), 0, n - 1)
        beam_range = np.asarray(ranges, dtype=np.float64).ravel()
        if beam_range.size != n:
            beam_range = np.resize(beam_range, n)
        beam_range = np.clip(beam_range, 0.0, float(C.CAM_RANGE))
        hit = beam_range < float(C.CAM_RANGE) * 0.98
        nearest = beam_range[idx]
        nearest_hit = hit[idx]
        res = self.resolution
        free = in_fan & ((~nearest_hit) | (self._radius < nearest - 0.5 * res))
        occupied = in_fan & nearest_hit & (
            np.abs(self._radius - nearest) <= 1.5 * res)
        # the far end of a no-return ray is not evidence of empty space either
        free &= self._radius <= float(C.CAM_RANGE) * 0.98
        return free, occupied
