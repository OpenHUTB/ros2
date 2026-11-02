"""Perception back-ends that feed the active perception planner.

Two implementations are provided:

``DemoPerceptionModel``
    A deterministic, dependency-free stand-in for the segmentation network.  It
    synthesises a ground-truth semantic field over the operational area and
    reports the entropy of its own belief given the observations accumulated so
    far.  Its purpose is to make the whole AP-CPP loop runnable and testable
    without TensorFlow, GDAL, or an AirSim binary.

``SegmentationPerceptionModel``
    A thin adapter around the upstream ``GetViewpointImage`` / ``GetSpeed``
    pair.  It crops the orthomosaic at the current viewpoint and runs the
    ``segmentation_models`` U-Net, converting the softmax output into a
    normalised Shannon entropy over the class distribution - which is exactly
    the ``measurement_entropy`` the belief update consumes.

Both expose the same call signature, so swapping them is a one-line change in
the mission driver.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from ap_cpp.grid_model import CoverageGrid
from ap_cpp.pose import Pose
from ap_cpp.sensor import SensorModel

__all__ = [
    "PerceptionReading",
    "PerceptionModel",
    "DemoPerceptionModel",
    "SegmentationPerceptionModel",
]


@dataclass
class PerceptionReading:
    """What a perception back-end reports for one frame."""

    measurement_entropy: float
    """Normalised Shannon entropy over the semantic classes, in ``[0, 1]``."""

    coverage_ratio: float
    """Fraction of the frame assigned to a vegetated class, in ``[0, 1]``.

    This is the same quantity ``GetSpeed.get_coverage_ratio`` computes, kept
    so the AP-CPP speed channel stays numerically compatible with the baseline.
    """

    confidence: float
    """``1 - measurement_entropy``; the ``confidence level`` of the original G(x, y)."""

    dominant_class: Optional[int] = None
    entropy_nats: Optional[float] = None


class PerceptionModel:
    """Interface implemented by every perception back-end."""

    def perceive(self, grid: CoverageGrid, pose: Pose, t: float) -> PerceptionReading:
        raise NotImplementedError


class DemoPerceptionModel(PerceptionModel):
    """Ground-truth-driven synthetic perception.

    Constructs a hidden semantic map with three zones (bare soil, crop, weed
    patch) plus a couple of anomaly blobs.  For a requested pose it returns the
    entropy of the *class distribution* observed inside the sensor footprint,
    which falls as the accumulated coverage of those cells grows: an area that
    the robot has already imaged confidently no longer surprises the model.
    """

    NAMES = ("soil", "crop", "weed")

    def __init__(
        self,
        grid: CoverageGrid,
        sensor: SensorModel,
        seed: int = 7,
        noise: float = 0.03,
    ):
        self.grid = grid
        self.sensor = sensor
        self.noise = float(noise)
        self.rng = np.random.default_rng(seed)
        self.truth = self._build_truth()
        self._cell_class_probs = self._build_class_probs()

    # -- ground truth ------------------------------------------------------ #
    def _build_truth(self) -> np.ndarray:
        """Integer semantic map: 0 soil, 1 crop, 2 weed.

        Vegetation covers roughly a fifth of the field.  That density matters:
        the ``GetSpeed`` law inherited from upstream is calibrated for a
        vegetated fraction of 5-15% (``ratio_min``/``ratio_max`` in
        ``get_new_speed.py``), so a synthetic field that is mostly crop would
        saturate the controller at its slowest setting and hide the very
        behaviour the demo is meant to exercise.
        """
        nx, ny = self.grid.nx, self.grid.ny
        centres = self.grid.cell_centers().reshape(nx, ny, 2)
        x = centres[:, :, 0]
        y = centres[:, :, 1]

        span_x = max(self.grid.config.x_max - self.grid.config.x_min, 1e-6)
        span_y = max(self.grid.config.y_max - self.grid.config.y_min, 1e-6)
        u = (x - self.grid.config.x_min) / span_x
        v = (y - self.grid.config.y_min) / span_y

        truth = np.zeros((nx, ny), dtype=np.int32)

        # A narrow crop band running diagonally across the field.
        band = np.abs((u - 0.5) * 1.4 + (v - 0.5) * 0.6)
        truth[band < 0.09] = 1

        # Two weed patches: one on the band, one in otherwise clean soil.
        d1 = (u - 0.32) ** 2 + (v - 0.68) ** 2
        d2 = (u - 0.76) ** 2 + (v - 0.26) ** 2
        truth[d1 < 0.006] = 2
        truth[d2 < 0.004] = 2

        truth[~self.grid.passable] = 0
        return truth

    def _build_class_probs(self) -> np.ndarray:
        """Per-cell class distribution that an *oracle, fully-informed* model
        would report.  Mixing this with the uniform prior by accumulated
        coverage gives a smoothly degrading belief without any training."""
        nx, ny = self.grid.nx, self.grid.ny
        probs = np.full((nx, ny, 3), 1.0 / 3.0, dtype=np.float64)
        for cls in range(3):
            probs[:, :, cls] = np.where(self.truth == cls, 0.86, 0.07)
        probs = probs / probs.sum(axis=2, keepdims=True)
        return probs

    # -- interface --------------------------------------------------------- #
    def perceive(self, grid: CoverageGrid, pose: Pose, t: float) -> PerceptionReading:
        cells, weights, etas = self.sensor.footprint(grid, pose)
        if not cells:
            return PerceptionReading(
                measurement_entropy=1.0, coverage_ratio=0.0, confidence=0.0
            )

        ii = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
        jj = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))

        # The segmentation network's output entropy is a property of the
        # *image*, not of the robot's belief.  Sensor-geometry penalties
        # (off-nadir blur, illumination) are applied once, downstream, as the
        # per-cell ``eta`` weight of the belief update - folding them in here
        # as well would charge the robot twice for looking at the frame edge.
        w = np.asarray(weights, dtype=np.float64)
        probs = self._cell_class_probs[ii, jj] + self.rng.normal(
            0.0, self.noise, (len(cells), 3)
        )
        probs = np.clip(probs, 1e-6, None)
        probs = probs / probs.sum(axis=1, keepdims=True)

        # Match the TensorFlow back-end: the frame score is the *mean per-pixel*
        # entropy, not the entropy of the frame-averaged class distribution.
        # Averaging the distribution first would report a mixed soil/crop frame
        # as near-maximally uncertain even when every pixel is classified
        # confidently.
        per_cell_entropy = -np.sum(probs * np.log(probs), axis=1) / math.log(3.0)
        measurement_entropy = float(np.sum(per_cell_entropy * w) / max(w.sum(), 1e-9))
        frame_probs = (probs * w[:, None]).sum(axis=0) / max(float(w.sum()), 1e-9)

        # Coverage ratio follows the upstream definition: fraction of the frame
        # in a vegetated class (crop or weed).  A real camera measures what is
        # actually there, so this reads the ground truth rather than the belief
        # - otherwise the speed channel would be a function of the map instead
        # of a function of the field.
        vegetation = (self.truth[ii, jj] > 0).astype(np.float64)
        coverage_ratio = float(np.sum(vegetation * weights))

        return PerceptionReading(
            measurement_entropy=float(np.clip(measurement_entropy, 0.0, 1.0)),
            coverage_ratio=coverage_ratio,
            confidence=float(np.clip(1.0 - measurement_entropy, 0.0, 1.0)),
            dominant_class=int(np.argmax(frame_probs)),
            entropy_nats=float(measurement_entropy * math.log(3.0)),
        )

    # -- scoring helpers --------------------------------------------------- #
    def zone_entropy(self, mask: np.ndarray) -> float:
        """Mean oracle entropy over a boolean cell mask (used by tests)."""
        sub = self._cell_class_probs[mask]
        if sub.size == 0:
            return 0.0
        h = -np.sum(sub * np.log(sub), axis=1) / math.log(3.0)
        return float(h.mean())


class SegmentationPerceptionModel(PerceptionModel):
    """Adapter for the upstream U-Net segmentation pipeline.

    Parameters are read from ``parameters.py`` exactly as ``main.py`` does, so
    the AP-CPP layer can be dropped into the existing AirSim pipeline without
    retraining or re-plumbing paths.

    Notes
    -----
    TensorFlow / GDAL are imported lazily inside ``__init__`` - importing this
    module must not require them, otherwise the demo could not run on a machine
    without the full simulation stack.
    """

    def __init__(self, weights_path: str = None, backbone: str = None):
        import get_new_speed  # noqa: F401  (validates the TF/GDAL stack)
        from get_new_speed import GetSpeed, GetViewpointImage

        if weights_path is None or backbone is None:
            from parameters import backbone as _bb, weights_path as _wp

            weights_path = weights_path or _wp
            backbone = backbone or _bb

        self.viewpoint = GetViewpointImage()
        self.network = GetSpeed(weights_path, backbone)

    def perceive(self, grid: CoverageGrid, pose: Pose, t: float) -> PerceptionReading:
        raise NotImplementedError(
            "SegmentationPerceptionModel.perceive needs a viewpoint in WGS84; "
            "use perceive_wgs84(waypoint, orientation_deg) from the AirSim driver."
        )

    def perceive_wgs84(
        self, waypoint: Sequence[float], orientation_deg: float
    ) -> PerceptionReading:
        """Run the network on the orthomosaic crop at ``waypoint``.

        ``waypoint`` is ``[lat, lon]`` (WGS84) and ``orientation_deg`` is the
        vehicle yaw, matching ``main.py``'s call into ``GetViewpointImage``.
        """
        img = self.viewpoint.get_image(list(waypoint), orientation_deg)
        probs = self.network.model.predict(np.expand_dims(
            self.network.preprocess_input(img), 0
        ))[0]

        eps = 1e-8
        p = np.clip(probs, eps, 1.0)
        p = p / p.sum(axis=-1, keepdims=True)
        entropy_nats = -np.sum(p * np.log(p), axis=-1)
        measurement_entropy = float(
            np.clip(entropy_nats / math.log(p.shape[-1]), 0.0, 1.0).mean()
        )

        hard = np.argmax(probs, axis=-1).ravel()
        coverage_ratio = float(
            (np.sum(hard == 1) + np.sum(hard == 2)) / max(hard.size, 1)
        )

        return PerceptionReading(
            measurement_entropy=measurement_entropy,
            coverage_ratio=coverage_ratio,
            confidence=float(np.clip(1.0 - measurement_entropy, 0.0, 1.0)),
            dominant_class=int(np.bincount(hard, minlength=p.shape[-1]).argmax()),
            entropy_nats=float(entropy_nats.mean()),
        )
