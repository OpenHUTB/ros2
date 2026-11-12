"""Occupancy mapping: from a predicted depth mask to a planning grid."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .sim.world import World


def _dilate(grid: np.ndarray, cells: int) -> np.ndarray:
    out = grid.copy()
    for _ in range(int(cells)):
        g = out.copy()
        out[1:, :] |= g[:-1, :]
        out[:-1, :] |= g[1:, :]
        out[:, 1:] |= g[:, :-1]
        out[:, :-1] |= g[:, 1:]
        out[1:, 1:] |= g[:-1, :-1]
        out[:-1, :-1] |= g[1:, 1:]
        out[1:, :-1] |= g[:-1, 1:]
        out[:-1, 1:] |= g[1:, :-1]
    return out


def patch_clearance(patch: np.ndarray, res: float = 0.5) -> float:
    """Distance (m) from the patch centre to the closest occupied cell.

    Used by the learned controller instead of the ground-truth world clearance,
    so that inference only consumes what an onboard map would provide.
    """
    occ = np.argwhere(np.asarray(patch) > 0.5)
    if len(occ) == 0:
        return float(patch.shape[0] * res * 0.7)
    c = np.array([patch.shape[0] // 2, patch.shape[1] // 2])
    return float(np.min(np.linalg.norm(occ - c, axis=1)) * res)


@dataclass
class GridMap:
    """A 2-D occupancy grid in world coordinates (ROS REP-103)."""

    res: float = 0.5
    x0: float = 0.0
    y0: float = -22.0
    nx: int = 104
    ny: int = 88
    occ: Optional[np.ndarray] = None
    hits: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.occ is None:
            self.occ = np.zeros((self.nx, self.ny), dtype=bool)
        if self.hits is None:
            self.hits = np.zeros((self.nx, self.ny), dtype=np.float32)

    @classmethod
    def from_world(cls, world: World, res: float = 0.5, inflate: float = 0.45) -> "GridMap":
        x0, x1, y0, y1 = world.bounds
        nx = int(round((x1 - x0) / res))
        ny = int(round((y1 - y0) / res))
        m = cls(res=res, x0=x0, y0=y0, nx=nx, ny=ny)
        m.occ = world.occupancy(x0, x0 + nx * res, y0, y0 + ny * res, res,
                                world.flight_z, inflation=inflate)
        m.hits[:] = m.occ.astype(np.float32) * 10.0
        return m

    # ------------------------------------------------------------- coordinates
    def to_cell(self, p) -> Tuple[int, int]:
        ix = int(math.floor((float(p[0]) - self.x0) / self.res))
        iy = int(math.floor((float(p[1]) - self.y0) / self.res))
        return min(max(ix, 0), self.nx - 1), min(max(iy, 0), self.ny - 1)

    def to_world(self, ix: int, iy: int) -> np.ndarray:
        return np.array([self.x0 + (ix + 0.5) * self.res,
                         self.y0 + (iy + 0.5) * self.res, 3.0])

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # ------------------------------------------------------------------ update
    def decay(self, factor: float = 0.99):
        self.hits *= factor
        self.occ = self.hits > 0.6

    def integrate_depth(self, depth: np.ndarray, prob: np.ndarray,
                        pos: np.ndarray, yaw: float, fov_deg: float = 90.0,
                        max_range: float = 12.0, threshold: float = 0.55,
                        radius: float = 0.30) -> int:
        """Project pixels classified as obstacles into the global grid."""
        h, w = depth.shape
        f = 0.5 * h / math.tan(math.radians(fov_deg) * 0.5)
        us = (np.arange(w) + 0.5 - w / 2) / f
        vs = (np.arange(h) + 0.5 - h / 2) / f
        uu, vv = np.meshgrid(us, vs)
        norm = np.sqrt(uu * uu + vv * vv + 1.0)
        dir_z = -vv / norm
        mask = (prob > threshold) & (depth > 0.4) & (depth < max_range - 1e-3)
        # reject ground returns: the projected point must be above 0.55 m
        z = pos[2] + depth * dir_z
        mask &= z > 0.55
        if not np.any(mask):
            return 0
        # Camera convention: column u grows to the right of the body x axis.
        # ``us`` is already normalised by the focal length, so the bearing offset
        # is simply atan(u).
        angles = np.arctan(uu)
        bearing = yaw + angles
        r = depth[mask]
        bx = pos[0] + r * np.cos(bearing[mask])
        by = pos[1] + r * np.sin(bearing[mask])
        ix = np.floor((bx - self.x0) / self.res).astype(int)
        iy = np.floor((by - self.y0) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        if not np.any(ok):
            return 0
        np.add.at(self.hits, (ix[ok], iy[ok]), 1.6)
        # Slow decay = short-term memory of obstacles that left the FOV.
        self.decay(0.99)
        self.occ = self.hits > 0.6
        return int(np.count_nonzero(ok))

    def integrate_points(self, pts: np.ndarray, radius: float = 0.25):
        if len(pts) == 0:
            return
        ix = np.floor((pts[:, 0] - self.x0) / self.res).astype(int)
        iy = np.floor((pts[:, 1] - self.y0) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        self.hits[ix[ok], iy[ok]] += 2.0
        self.occ = self.hits > 0.55

    # ------------------------------------------------------------------- query
    def plan_grid(self, margin: int = 1) -> np.ndarray:
        """Occupancy grid dilated by ``margin`` cells, used by the planner."""
        return _dilate(self.occ, margin) if margin > 0 else self.occ.copy()

    def free_mask(self, margin: int = 1) -> np.ndarray:
        return ~self.plan_grid(margin)

    def local_patch(self, p, cells: int = 5, res: Optional[float] = None) -> np.ndarray:
        """Occupancy patch around ``p`` (1 = occupied, 0 = free)."""
        res = res or self.res
        ix, iy = self.to_cell(p)
        r = cells // 2
        out = np.ones((cells, cells), dtype=np.float32)
        for i in range(cells):
            for j in range(cells):
                gx, gy = ix + i - r, iy + j - r
                if 0 <= gx < self.nx and 0 <= gy < self.ny:
                    out[i, j] = 1.0 if self.occ[gx, gy] else 0.0
        return out
