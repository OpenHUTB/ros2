"""Occupancy / information-entropy grid used by the active perception planner.

The grid is a *belief* over the operational area, expressed in the local NED
frame already used throughout ``handleGeo``.  Two fields are maintained per
cell:

``coverage``
    Accumulated observation quality in ``[0, 1]``.  ``1.0`` means the cell has
    been imaged often enough, and at a high enough ground sampling distance,
    to consider the coverage task satisfied for that cell.  This is the direct
    generalisation of the scalar ``coverage_ratio`` used by the original
    ``GetSpeed`` heuristic.

``entropy``
    Normalised Shannon entropy of the *semantic* belief in ``[0, 1]``.
    ``1.0`` means "we know nothing about what is in this cell", ``0.0`` means
    "the segmentation model is certain".  This is the quantity the active
    perception layer is trying to drive down, and it corresponds to the
    ``confidence level`` term of the original ``G(x, y)`` speed function.

Cells that are not traversable (outside the polygon, or inside an obstacle)
carry ``entropy = 0`` and ``coverage = 1`` so that they never attract
information-gain driven motion, while remaining valid for path search.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["GridConfig", "CoverageGrid"]


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class GridConfig:
    """Raster geometry of the operational area.

    Parameters
    ----------
    resolution:
        Cell edge length in metres.
    x_min, y_min, x_max, y_max:
        Extent of the raster in the local NED frame (metres).  Cells are
        ``resolution`` metres wide, so the raster shape is ``ceil`` of the
        span over the resolution.
    initial_entropy:
        Prior entropy assigned to every traversable cell before any
        observation.  ``1.0`` is a uniform, uninformative prior.
    obstacle_entropy, obstacle_coverage:
        Sentinel values written into non-traversable cells.
    """

    resolution: float = 2.5
    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 100.0
    y_max: float = 100.0
    initial_entropy: float = 1.0
    obstacle_entropy: float = 0.0
    obstacle_coverage: float = 1.0

    def __post_init__(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("grid extent must be non-degenerate")

    @property
    def nx(self) -> int:
        """Number of cells along the north axis."""
        return int(np.ceil((self.x_max - self.x_min) / self.resolution))

    @property
    def ny(self) -> int:
        """Number of cells along the east axis."""
        return int(np.ceil((self.y_max - self.y_min) / self.resolution))

    @property
    def shape(self) -> Tuple[int, int]:
        return (self.nx, self.ny)

    @property
    def cell_area(self) -> float:
        return float(self.resolution ** 2)

    def offset(self, margin: float) -> "GridConfig":
        """Return a copy inflated by ``margin`` metres on every side."""
        return GridConfig(
            resolution=self.resolution,
            x_min=self.x_min - margin,
            y_min=self.y_min - margin,
            x_max=self.x_max + margin,
            y_max=self.y_max + margin,
            initial_entropy=self.initial_entropy,
            obstacle_entropy=self.obstacle_entropy,
            obstacle_coverage=self.obstacle_coverage,
        )

    @classmethod
    def from_polygon(
        cls,
        polygon_xy: Sequence[Sequence[float]],
        resolution: float = 2.5,
        margin: float = 0.0,
        **kwargs,
    ) -> "GridConfig":
        """Build a raster whose extent is the bounding box of ``polygon_xy``."""
        if len(polygon_xy) < 3:
            raise ValueError("polygon needs at least three vertices")
        pts = np.asarray(polygon_xy, dtype=float)
        return cls(
            resolution=resolution,
            x_min=float(pts[:, 0].min()) - margin,
            y_min=float(pts[:, 1].min()) - margin,
            x_max=float(pts[:, 0].max()) + margin,
            y_max=float(pts[:, 1].max()) + margin,
            **kwargs,
        )


# --------------------------------------------------------------------------- #
# Grid
# --------------------------------------------------------------------------- #
class CoverageGrid:
    """Belief raster + traversal bookkeeping for a coverage mission."""

    def __init__(self, config: GridConfig):
        self.config = config
        self.nx, self.ny = config.shape

        self.coverage = np.zeros((self.nx, self.ny), dtype=np.float64)
        self.entropy = np.full(
            (self.nx, self.ny), config.initial_entropy, dtype=np.float64
        )
        self.visits = np.zeros((self.nx, self.ny), dtype=np.int32)
        self.last_seen = np.full((self.nx, self.ny), -1.0, dtype=np.float64)

        #: Every cell is traversable until a geometry rasterisation says otherwise.
        self.passable = np.ones((self.nx, self.ny), dtype=bool)

        #: Optional application-specific prior overlaid on ``entropy`` (e.g. an
        #: agronomic risk map or an NDVI anomaly layer).
        self.uncertainty_prior: Optional[np.ndarray] = None
        self._prior_weight = 0.0

    # -- construction ------------------------------------------------------ #
    def set_uncertainty_prior(
        self, prior: np.ndarray, weight: float = 0.3
    ) -> None:
        """Blend an external risk layer into the initial entropy field.

        The prior is expected in ``[0, 1]`` and is clipped and resampled-free:
        it must already match the raster shape.
        """
        prior = np.asarray(prior, dtype=np.float64)
        if prior.shape != (self.nx, self.ny):
            raise ValueError(
                "uncertainty prior shape {} does not match raster {}".format(
                    prior.shape, (self.nx, self.ny)
                )
            )
        self.uncertainty_prior = np.clip(prior, 0.0, 1.0)
        self._prior_weight = float(np.clip(weight, 0.0, 1.0))
        self.entropy = np.clip(
            (1.0 - self._prior_weight) * self.entropy
            + self._prior_weight * self.uncertainty_prior,
            0.0,
            1.0,
        )

    # -- geometry ---------------------------------------------------------- #
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Continuous NED position -> integer cell index. May fall outside."""
        cfg = self.config
        i = int(np.floor((x - cfg.x_min) / cfg.resolution))
        j = int(np.floor((y - cfg.y_min) / cfg.resolution))
        return i, j

    def cell_to_world(self, i: int, j: int) -> Tuple[float, float]:
        """Lower-left corner of cell ``(i, j)`` in NED metres."""
        cfg = self.config
        return (cfg.x_min + i * cfg.resolution, cfg.y_min + j * cfg.resolution)

    def cell_center(self, i: int, j: int) -> Tuple[float, float]:
        """Centre of cell ``(i, j)`` in NED metres."""
        x, y = self.cell_to_world(i, j)
        half = 0.5 * self.config.resolution
        return (x + half, y + half)

    def contains(self, i: int, j: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny

    def nearest_free_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Snap an arbitrary NED point to the closest traversable cell.

        Used to project a continuously-planned waypoint back onto the graph.
        Returns ``None`` when no traversable cell exists at all.
        """
        i, j = self.world_to_cell(x, y)
        if self.contains(i, j) and self.passable[i, j]:
            return i, j
        if not self.passable.any():
            return None
        free = np.argwhere(self.passable)
        centres = np.column_stack(
            (
                self.config.x_min + (free[:, 0] + 0.5) * self.config.resolution,
                self.config.y_min + (free[:, 1] + 0.5) * self.config.resolution,
            )
        )
        k = int(np.argmin(np.sum((centres - np.array([x, y])) ** 2, axis=1)))
        return int(free[k, 0]), int(free[k, 1])

    # -- rasterisation ----------------------------------------------------- #
    def rasterize_polygon(self, polygon_xy: Sequence[Sequence[float]]) -> None:
        """Mark cells whose centre lies inside ``polygon_xy`` as traversable."""
        inside = self._points_in_polygon(self.cell_centers(), polygon_xy)
        self.passable &= inside.reshape(self.nx, self.ny)
        self._apply_obstacle_sentinels()

    def rasterize_obstacles(
        self, obstacles_xy: Iterable[Sequence[Sequence[float]]]
    ) -> None:
        """Carve obstacle polygons out of the traversable set."""
        for poly in obstacles_xy:
            poly = list(poly)
            if len(poly) < 3:
                continue
            inside = self._points_in_polygon(self.cell_centers(), poly)
            self.passable &= ~inside.reshape(self.nx, self.ny)
        self._apply_obstacle_sentinels()

    def _apply_obstacle_sentinels(self) -> None:
        blocked = ~self.passable
        self.entropy[blocked] = self.config.obstacle_entropy
        self.coverage[blocked] = self.config.obstacle_coverage

    def cell_centers(self) -> np.ndarray:
        """``(nx * ny, 2)`` array of cell centres, row-major over ``(i, j)``."""
        xs = self.config.x_min + (np.arange(self.nx) + 0.5) * self.config.resolution
        ys = self.config.y_min + (np.arange(self.ny) + 0.5) * self.config.resolution
        grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
        return np.column_stack((grid_x.ravel(), grid_y.ravel()))

    @staticmethod
    def _points_in_polygon(points: np.ndarray, polygon_xy) -> np.ndarray:
        """Vectorised even-odd ray casting for every point at once."""
        poly = np.asarray(polygon_xy, dtype=float)
        px = points[:, 0]
        py = points[:, 1]
        inside = np.zeros(points.shape[0], dtype=bool)

        n = poly.shape[0]
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            crosses = (yi > py) != (yj > py)
            denom = yj - yi
            denom = denom if abs(denom) > 1e-12 else 1e-12
            x_int = (xj - xi) * (py - yi) / denom + xi
            inside ^= crosses & (px < x_int)
            j = i
        return inside

    # -- traversal --------------------------------------------------------- #
    def neighbors(
        self, cell: Tuple[int, int], connectivity: int = 8
    ) -> List[Tuple[int, int]]:
        """Traversable neighbours of ``cell``.

        For 8-connectivity a diagonal move is rejected when either of the two
        orthogonal cells that the move grazes is blocked, which keeps the
        emitted paths free of corner cutting.
        """
        i, j = cell
        out: List[Tuple[int, int]] = []
        orthogonal = ((1, 0), (-1, 0), (0, 1), (0, -1))
        diagonal = ((1, 1), (1, -1), (-1, 1), (-1, -1))

        for di, dj in orthogonal:
            ni, nj = i + di, j + dj
            if self.contains(ni, nj) and self.passable[ni, nj]:
                out.append((ni, nj))

        if connectivity == 8:
            for di, dj in diagonal:
                ni, nj = i + di, j + dj
                if not (self.contains(ni, nj) and self.passable[ni, nj]):
                    continue
                if not (self.passable[i + di, j] and self.passable[i, j + dj]):
                    continue
                out.append((ni, nj))
        return out

    def ray_cells(self, p0, p1) -> List[Tuple[int, int]]:
        """Supercover line from ``p0`` to ``p1`` in NED metres."""
        x0, y0 = float(p0[0]), float(p0[1])
        x1, y1 = float(p1[0]), float(p1[1])
        length = float(np.hypot(x1 - x0, y1 - y0))
        steps = max(int(np.ceil(length / (0.5 * self.config.resolution))), 1)
        ts = np.linspace(0.0, 1.0, steps + 1)
        xs = x0 + ts * (x1 - x0)
        ys = y0 + ts * (y1 - y0)
        cells = []
        seen = set()
        for x, y in zip(xs, ys):
            cell = self.world_to_cell(x, y)
            if cell in seen:
                continue
            seen.add(cell)
            cells.append(cell)
        return cells

    def has_line_of_sight(self, p0, p1) -> bool:
        """True when no blocked cell lies on the segment ``p0 -> p1``."""
        for i, j in self.ray_cells(p0, p1):
            if not self.contains(i, j):
                return False
            if not self.passable[i, j]:
                # Endpoints may legitimately sit on a boundary cell.
                if (i, j) != self.world_to_cell(*p1):
                    return False
        return True

    # -- belief update ----------------------------------------------------- #
    @staticmethod
    def fuse(
        cells: Sequence[Tuple[int, int]],
        prior_entropy: np.ndarray,
        prior_coverage: Optional[np.ndarray],
        measurement_entropy: float,
        weights: Optional[Sequence[float]] = None,
        etas: Optional[Sequence[float]] = None,
        coverage_gain: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
        """Pure, side-effect-free belief fusion.

        Both the online update (:meth:`observe`) and the planning-time
        prediction used by the utility model route through this function, so a
        planned observation is scored with exactly the update that will later
        be applied.

        Returns
        -------
        ui, uj:
            Row/column indices of the observed cells.
        posterior:
            Updated entropy for those cells.
        posterior_cov:
            Updated coverage for those cells.
        gain:
            Total entropy reduction.
        frame_eta:
            Frame-averaged observation efficiency (drives the coverage update).
        """
        idx = np.asarray(cells, dtype=np.int64)
        if idx.size == 0:
            empty = np.zeros(0, dtype=np.int64)
            return empty, empty, np.zeros(0), np.zeros(0), 0.0, 0.0

        ui, uj = idx[:, 0], idx[:, 1]
        if weights is None:
            w = np.ones(ui.size, dtype=np.float64)
        else:
            w = np.clip(np.asarray(weights, dtype=np.float64), 0.0, 1.0)
        if etas is None:
            e = np.ones(ui.size, dtype=np.float64)
        else:
            e = np.clip(np.asarray(etas, dtype=np.float64), 0.0, 1.0)

        meas_h = float(np.clip(measurement_entropy, 0.0, 1.0))
        prior = prior_entropy[ui, uj]

        # Bayesian-style multiplicative fusion: a perfect measurement of a
        # deterministic cell (measurement_entropy -> 0) collapses the belief,
        # an uninformative one (measurement_entropy -> 1) leaves it untouched.
        informativeness = np.clip(w * e * (1.0 - meas_h), 0.0, 1.0)
        posterior = np.clip(prior * (1.0 - informativeness), 0.0, 1.0)
        gain = float(np.sum(prior - posterior))

        frame_eta = float(e.mean()) if e.size else 0.0
        if prior_coverage is None:
            posterior_cov = np.zeros(0, dtype=np.float64)
        else:
            cov = prior_coverage[ui, uj]
            posterior_cov = np.clip(
                cov + frame_eta * coverage_gain * (1.0 - cov), 0.0, 1.0
            )

        return ui, uj, posterior, posterior_cov, gain, frame_eta

    def observe(
        self,
        cells: Sequence[Tuple[int, int]],
        measurement_entropy: float,
        weights: Optional[Sequence[float]] = None,
        etas: Optional[Sequence[float]] = None,
        coverage_gain: float = 1.0,
        timestep: float = 0.0,
    ) -> float:
        """Fuse one observation into the belief.

        Parameters
        ----------
        cells:
            Cells touched by the sensor footprint.
        measurement_entropy:
            Normalised entropy the segmentation model reports for this frame,
            in ``[0, 1]``.
        weights:
            Per-cell in-frame weighting (e.g. a Gaussian falloff about the
            optical axis).  Defaults to uniform.
        etas:
            Per-cell observation efficiency, folding in altitude, off-nadir
            angle, illumination and motion blur.  Defaults to unity.
        coverage_gain:
            Per-observation coverage increment before the efficiency weighting.
        timestep:
            Mission time stamp recorded in ``last_seen``.

        Returns
        -------
        float
            Total entropy reduction achieved across ``cells``.  This is exactly
            the quantity the rolling-horizon utility maximises.
        """
        if len(cells) == 0:
            return 0.0

        idx = np.asarray(cells, dtype=np.int64)
        if idx.ndim != 2 or idx.shape[1] != 2:
            return 0.0

        valid = (
            (idx[:, 0] >= 0)
            & (idx[:, 0] < self.nx)
            & (idx[:, 1] >= 0)
            & (idx[:, 1] < self.ny)
        )
        idx = idx[valid]
        if idx.size == 0:
            return 0.0

        traversable = self.passable[idx[:, 0], idx[:, 1]]
        idx = idx[traversable]
        if idx.size == 0:
            return 0.0

        w = None if weights is None else np.asarray(weights, dtype=np.float64)[valid][traversable]
        e = None if etas is None else np.asarray(etas, dtype=np.float64)[valid][traversable]

        ui, uj, posterior, posterior_cov, gain, _ = self.fuse(
            idx,
            prior_entropy=self.entropy,
            prior_coverage=self.coverage,
            measurement_entropy=measurement_entropy,
            weights=w,
            etas=e,
            coverage_gain=coverage_gain,
        )
        if ui.size == 0:
            return 0.0

        self.entropy[ui, uj] = posterior
        self.coverage[ui, uj] = posterior_cov
        np.add.at(self.visits, (ui, uj), 1)
        self.last_seen[ui, uj] = timestep
        return gain

    # -- queries ----------------------------------------------------------- #
    def free_mask(self) -> np.ndarray:
        return self.passable

    def free_cells(self) -> np.ndarray:
        """``(N, 2)`` integer array of traversable cell indices."""
        return np.argwhere(self.passable)

    def frontier_cells(self, entropy_threshold: float = 0.5) -> np.ndarray:
        """Traversable, still-uncertain cells - the active perception targets."""
        return np.argwhere(self.passable & (self.entropy >= entropy_threshold))

    def mean_entropy(self) -> float:
        free = self.passable
        if not free.any():
            return 0.0
        return float(self.entropy[free].mean())

    def total_entropy(self) -> float:
        return float(self.entropy[self.passable].sum())

    def coverage_fraction(self) -> float:
        free = self.passable
        if not free.any():
            return 1.0
        return float(self.coverage[free].mean())

    def mission_complete(self, coverage_target: float, entropy_target: float) -> bool:
        return (
            self.coverage_fraction() >= coverage_target
            and self.mean_entropy() <= entropy_target
        )

    def entropy_in_radius(self, center, radius: float) -> float:
        """Mean entropy within ``radius`` metres of ``center`` (a proxy for
        the information a dwell at that pose is expected to yield)."""
        c = np.asarray(center, dtype=float)
        centres = self.cell_centers()
        d2 = np.sum((centres - c) ** 2, axis=1)
        sel = (d2 <= radius ** 2).reshape(self.nx, self.ny) & self.passable
        if not sel.any():
            return 0.0
        return float(self.entropy[sel].mean())

    # -- reporting --------------------------------------------------------- #
    def summary(self) -> dict:
        return {
            "shape": (self.nx, self.ny),
            "resolution_m": self.config.resolution,
            "free_cells": int(self.passable.sum()),
            "coverage_fraction": self.coverage_fraction(),
            "mean_entropy": self.mean_entropy(),
            "total_entropy": self.total_entropy(),
            "frontier_cells": int(self.frontier_cells().shape[0]),
            "visited_cells": int((self.visits > 0).sum()),
        }

    def snapshot(self) -> dict:
        """Deep copy of the mutable belief arrays (for logging / replay)."""
        return {
            "coverage": self.coverage.copy(),
            "entropy": self.entropy.copy(),
            "visits": self.visits.copy(),
            "last_seen": self.last_seen.copy(),
            "passable": self.passable.copy(),
        }
