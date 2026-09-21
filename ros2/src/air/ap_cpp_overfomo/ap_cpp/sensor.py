"""Downward-looking camera model driving the active perception loop.

The sensor is modelled as an oriented ground rectangle (the projection of the
image plane at the flight altitude) with a cosine falloff of observation
efficiency towards the frame edges.  That falloff is the physical reason a
coverage mission cannot simply ignore where it looks: two cells at the same
distance from the robot may carry very different information value depending
on whether they fall at the centre or the periphery of the frame.

Default parameters match the RedEdge-M payload already used by ``main.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from ap_cpp.grid_model import CoverageGrid
from ap_cpp.pose import Pose

__all__ = ["SensorConfig", "SensorModel"]


@dataclass
class SensorConfig:
    """Geometric and radiometric properties of the imaging payload."""

    hfov_deg: float = 46.0
    """Horizontal field of view. RedEdge-M default is 46 deg."""

    vfov_deg: float = 35.0
    """Vertical field of view (derived from the 4:3 image aspect)."""

    altitude: float = 10.0
    """Above-ground flight altitude in metres."""

    eta_nominal: float = 0.85
    """Peak observation efficiency at the nadir point, in ``[0, 1]``."""

    min_eta: float = 0.15
    """Floor below which a cell is considered unobserved (frame periphery)."""

    coverage_gain: float = 0.6
    """Coverage increment credited per observation of a cell."""

    footprint_sharpness: float = 1.4
    """How aggressive the in-frame cosine falloff is (1.0 = pure geometry)."""

    @property
    def ground_width(self) -> float:
        """Swath width - footprint extent *across* the flight direction.

        This is the quantity that sets the lane spacing of a lawnmower sweep.
        For a nadir mapping camera the image's horizontal axis (and therefore
        the 46 deg HFOV the RedEdge-M is specified with) lies across track.
        """
        return 2.0 * self.altitude * math.tan(math.radians(self.hfov_deg / 2.0))

    @property
    def ground_height(self) -> float:
        """Footprint extent *along* the flight direction, in metres.

        Sets the along-track sampling: observation poses spaced further apart
        than this start to leave gaps.
        """
        return 2.0 * self.altitude * math.tan(math.radians(self.vfov_deg / 2.0))

    @property
    def max_ground_radius(self) -> float:
        """Half-diagonal of the footprint - the useful sensing radius."""
        return 0.5 * math.hypot(self.ground_width, self.ground_height)

    def lane_spacing(self, side_lap_percent: float) -> float:
        """Swath spacing for a given side lap, in metres.

        Reproduces the ``scanDist`` computation in ``main.py``
        (``covered(altitude) * (1 - sidelap / 100)``) with the cross-track
        extent, which is what a lawnmower sweep actually needs to overlap.
        """
        return self.ground_width * (1.0 - float(side_lap_percent) / 100.0)


class SensorModel:
    """Computes footprints, per-cell efficiency and predicted information gain."""

    def __init__(self, config: SensorConfig = None):
        self.config = config or SensorConfig()

    # -- geometry ---------------------------------------------------------- #
    def footprint_bounds(self, pose: Pose):
        """Axis-aligned bounding box ``(ax_min, ay_min, ax_max, ay_max)`` of the
        oriented footprint - used to limit the cell search to a small window."""
        r = self.config.max_ground_radius
        return (pose.x - r, pose.y - r, pose.x + r, pose.y + r)

    def footprint(
        self, grid: CoverageGrid, pose: Pose
    ) -> Tuple[List[Tuple[int, int]], np.ndarray, np.ndarray]:
        """Rasterise the sensor footprint onto ``grid``.

        Returns
        -------
        cells:
            Traversable cells covered by the frame.
        weights:
            Normalised in-frame weighting per cell (Gaussian proximity to the
            optical axis); sums to 1.
        etas:
            Observation efficiency per cell after the cosine falloff.
        """
        c = self.config
        # ``u`` runs along the heading (body x), ``v`` across it (body y).
        # The swath (cross-track) extent therefore bounds ``v``.
        half_u = 0.5 * c.ground_height
        half_v = 0.5 * c.ground_width
        radius = c.max_ground_radius

        # Cheap rejection: candidate window around the pose.
        i0, j0 = grid.world_to_cell(pose.x - radius, pose.y - radius)
        i1, j1 = grid.world_to_cell(pose.x + radius, pose.y + radius)
        i0 = max(i0 - 1, 0)
        j0 = max(j0 - 1, 0)
        i1 = min(i1 + 1, grid.nx)
        j1 = min(j1 + 1, grid.ny)
        if i1 <= i0 or j1 <= j0:
            return [], np.zeros(0), np.zeros(0)

        sub = grid.passable[i0:i1, j0:j1]
        if not sub.any():
            return [], np.zeros(0), np.zeros(0)

        si, sj = np.nonzero(sub)
        ci = si + i0
        cj = sj + j0
        cx = grid.config.x_min + (ci + 0.5) * grid.config.resolution
        cy = grid.config.y_min + (cj + 0.5) * grid.config.resolution

        # Rotate into the frame-aligned coordinates. The frame is yawed with the
        # vehicle, so a body-frame axis u points along the heading.
        psi = math.radians(pose.yaw_deg)
        dx = cx - pose.x
        dy = cy - pose.y
        u = dx * math.cos(psi) + dy * math.sin(psi)
        v = -dx * math.sin(psi) + dy * math.cos(psi)

        inside = (np.abs(u) <= half_u) & (np.abs(v) <= half_v)
        if not inside.any():
            return [], np.zeros(0), np.zeros(0)

        u = u[inside]
        v = v[inside]
        ci = ci[inside]
        cj = cj[inside]

        # Normalised radial coordinate inside the frame, 0 at nadir, 1 at the
        # frame corner. Efficiency falls off as cos(off-nadir) ~ 1/sqrt(1+t^2).
        nu = u / max(half_u, 1e-9)
        nv = v / max(half_v, 1e-9)
        t2 = nu ** 2 + nv ** 2
        eta = c.eta_nominal / np.sqrt(1.0 + t2) ** c.footprint_sharpness
        keep = eta >= c.min_eta
        if not keep.any():
            return [], np.zeros(0), np.zeros(0)

        ci = ci[keep]
        cj = cj[keep]
        u = u[keep]
        v = v[keep]
        eta = eta[keep]

        # Gaussian in-frame weighting: the segmentation model is most reliable
        # near the optical axis.
        weights = np.exp(-2.0 * ((u / max(half_u, 1e-9)) ** 2 + (v / max(half_v, 1e-9)) ** 2))
        total = float(weights.sum())
        if total <= 0.0:
            weights = np.ones_like(weights)
            total = float(weights.sum())
        weights = weights / total

        cells = [(int(a), int(b)) for a, b in zip(ci, cj)]
        return cells, weights, eta

    def efficiency_at(self, pose: Pose, cell_center) -> float:
        """Observation efficiency for a single cell seen from ``pose``."""
        c = self.config
        half_u = 0.5 * c.ground_height
        half_v = 0.5 * c.ground_width
        psi = math.radians(pose.yaw_deg)
        dx = cell_center[0] - pose.x
        dy = cell_center[1] - pose.y
        u = dx * math.cos(psi) + dy * math.sin(psi)
        v = -dx * math.sin(psi) + dy * math.cos(psi)
        if abs(u) > half_u or abs(v) > half_v:
            return 0.0
        t2 = (u / max(half_u, 1e-9)) ** 2 + (v / max(half_v, 1e-9)) ** 2
        eta = c.eta_nominal / math.sqrt(1.0 + t2) ** c.footprint_sharpness
        return float(eta) if eta >= c.min_eta else 0.0

    # -- prediction -------------------------------------------------------- #
    def predict_update(
        self,
        grid: CoverageGrid,
        pose: Pose,
        entropy: np.ndarray = None,
        coverage: np.ndarray = None,
        assumed_measurement_entropy: float = 0.0,
        coverage_gain: float = None,
    ):
        """Predict the belief update a future observation from ``pose`` yields.

        Runs the *same* fusion kernel as :meth:`CoverageGrid.observe` on a
        supplied belief snapshot, but never mutates it.  ``assumed_measurement_entropy``
        is the frame quality the planner expects to encounter; ``0.0`` is the
        optimistic case (a fully confident segmentation).

        Returns
        -------
        dict with keys ``cells``, ``weights``, ``etas``, ``gain``,
        ``posterior``, ``posterior_coverage`` and ``frame_eta``.
        """
        cells, weights, etas = self.footprint(grid, pose)
        if not cells:
            return {
                "cells": [],
                "weights": np.zeros(0),
                "etas": np.zeros(0),
                "gain": 0.0,
                "posterior": np.zeros(0),
                "posterior_coverage": np.zeros(0),
                "frame_eta": 0.0,
            }

        idx = np.asarray(cells, dtype=np.int64)
        H = grid.entropy if entropy is None else entropy
        C = grid.coverage if coverage is None else coverage

        ui, uj, posterior, posterior_cov, gain, frame_eta = grid.fuse(
            idx,
            prior_entropy=H,
            prior_coverage=C,
            measurement_entropy=assumed_measurement_entropy,
            weights=weights,
            etas=etas,
            coverage_gain=(
                self.config.coverage_gain if coverage_gain is None else coverage_gain
            ),
        )
        return {
            "cells": cells,
            "weights": weights,
            "etas": etas,
            "indices": (ui, uj),
            "gain": gain,
            "posterior": posterior,
            "posterior_coverage": posterior_cov,
            "frame_eta": frame_eta,
        }

    def predicted_information_gain(
        self,
        grid: CoverageGrid,
        pose: Pose,
        entropy: np.ndarray = None,
        assumed_measurement_entropy: float = 0.0,
    ) -> float:
        """Expected entropy reduction if the robot observes from ``pose``."""
        return self.predict_update(
            grid,
            pose,
            entropy=entropy,
            assumed_measurement_entropy=assumed_measurement_entropy,
        )["gain"]

    def predictedFrontierFraction(
        self, grid: CoverageGrid, pose: Pose, entropy_threshold: float = 0.5
    ) -> float:
        """Fraction of the footprint whose belief is still uncertain."""
        cells, _, _ = self.footprint(grid, pose)
        if not cells:
            return 0.0
        ii = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
        jj = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
        return float((grid.entropy[ii, jj] >= entropy_threshold).mean())

    # -- actuation --------------------------------------------------------- #
    def observe(
        self,
        grid: CoverageGrid,
        pose: Pose,
        measurement_entropy: float,
        timestep: float = 0.0,
    ) -> float:
        """Fuse a real observation taken at ``pose`` into ``grid``.

        ``measurement_entropy`` is the normalised entropy reported by the
        segmentation network for this frame; it is produced by the same model
        that the baseline pipeline uses for speed adaptation, so AP-CPP reuses
        the perception stack unchanged.
        """
        cells, weights, etas = self.footprint(grid, pose)
        if not cells:
            return 0.0
        return grid.observe(
            cells,
            measurement_entropy=measurement_entropy,
            weights=weights,
            etas=etas,
            coverage_gain=self.config.coverage_gain,
            timestep=timestep,
        )

    def footprint_area(self) -> float:
        return self.config.ground_width * self.config.ground_height


def nominal_sensor_from_parameters(altitude: float = None) -> SensorConfig:
    """Build a :class:`SensorConfig` from the RedEdge-M spec block in ``main.py``."""
    cfg = SensorConfig()
    if altitude is not None:
        cfg.altitude = float(altitude)
    return cfg
