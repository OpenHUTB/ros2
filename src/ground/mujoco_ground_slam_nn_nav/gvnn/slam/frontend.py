"""Online SLAM front-end: incremental odometry + correlative matching."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from ..config import LIDAR_RANGE_MAX, MAP_RESOLUTION
from ..geometry import matrix_to_pose, pose_to_matrix, relative_pose
from .occupancy import OccupancyMap, ScanMatcher


def compose_pose(base: Sequence[float], delta: Sequence[float]) -> np.ndarray:
    return matrix_to_pose(pose_to_matrix(base) @ pose_to_matrix(delta))


class SlamFrontend:
    """Maintains the map and the corrected ``map -> base_link`` pose.

    Odometry is used as a *relative* motion model: the previous corrected pose
    is propagated by the odometry increment, then refined by scan matching.
    This avoids accumulating the absolute odometry drift in the prior.
    """

    def __init__(self, width: float, height: float,
                 initial_pose: Optional[Sequence[float]] = None,
                 matcher_options: Optional[dict] = None,
                 resolution: float = MAP_RESOLUTION):
        self.grid = OccupancyMap(width, height, resolution)
        options = dict(stride=3, prior_translation=0.8, prior_rotation=0.6,
                       search_xy=0.20, search_yaw_deg=4.0)
        if matcher_options:
            options.update(matcher_options)
        self.matcher = ScanMatcher(self.grid, **options)
        self.pose = np.zeros(3) if initial_pose is None else np.asarray(initial_pose, dtype=np.float64)
        self.last_odom: Optional[np.ndarray] = None
        self.last_score = 0.0
        self.last_prediction = self.pose.copy()
        self.last_delta = np.zeros(3)
        self.matches = 0

    def reset(self, pose: Sequence[float]) -> None:
        self.grid.reset()
        self.pose = np.asarray(pose, dtype=np.float64).copy()
        self.last_odom = None
        self.last_score = 0.0
        self.matches = 0

    def update(self, odom_pose: Sequence[float], ranges: Sequence[float],
               match: bool = True) -> Tuple[np.ndarray, float]:
        odom = np.asarray(odom_pose, dtype=np.float64)
        if self.last_odom is None:
            predicted = self.pose.copy()
        else:
            delta = relative_pose(self.last_odom, odom)
            predicted = compose_pose(self.pose, delta)
        self.last_prediction = predicted.copy()
        score = 0.0
        if match and self.grid.updates >= 2:
            corrected, score = self.matcher.match(predicted, ranges)
            self.matches += 1
        else:
            corrected = predicted
        self.pose = np.asarray(corrected, dtype=np.float64)
        self.last_delta = self.pose - predicted
        self.last_score = float(score)
        self.grid.update(self.pose, ranges)
        self.last_odom = odom
        return self.pose.copy(), float(score)

    def map_to_odom(self, odom_pose: Sequence[float]) -> np.ndarray:
        return matrix_to_pose(pose_to_matrix(self.pose) @ np.linalg.inv(pose_to_matrix(odom_pose)))

    @property
    def coverage(self) -> float:
        return self.grid.coverage()

    def planning_maps(self, factor: int = 2):
        """Occupied / unknown masks at the A* resolution."""
        from ..geometry import downsample_bool

        occ = downsample_bool(self.grid.occupied_mask(), factor)
        unknown = downsample_bool(self.grid.unknown_mask(), factor)
        return occ, unknown
