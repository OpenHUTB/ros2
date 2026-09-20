"""Glue between WGS84 mission files and the local NED belief raster.

The upstream repository expresses everything - operation polygons, obstacles,
the pre-computed ``TurnWPs.txt`` route - in WGS84 and converts to the local NED
frame through :class:`handleGeo.ConvCoords.ConvCoords`.  AP-CPP keeps that
convention untouched: this module only adds the loading and round-tripping
helpers the planner needs, so a raster produced here can be overlaid on an
AirSim plot without any additional transformation.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ap_cpp.grid_model import CoverageGrid, GridConfig
from ap_cpp.pose import Pose

__all__ = ["GeoBridge", "load_qgis_polygon", "load_waypoints_txt"]


# --------------------------------------------------------------------------- #
# File loaders
# --------------------------------------------------------------------------- #
def load_qgis_polygon(path: str) -> Tuple[List[List[float]], List[List[List[float]]], str]:
    """Read a QGIS-exported GeoJSON file.

    Handles both the ``MultiPolygon`` and ``MultiLineString`` shapes that the
    ``CPP/<field>/Polygon<field>.geojson`` files use upstream.

    Returns
    -------
    polygon:
        ``[[lat, lon], ...]`` outer ring of the operational area.
    obstacles:
        List of ``[[lat, lon], ...]`` rings.
    name:
        The ``name`` property of the GeoJSON document.
    """
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)

    polygon: List[List[float]] = []
    obstacles: List[List[List[float]]] = []

    for feature in data.get("features", []):
        geom = feature.get("geometry", {})
        gtype = geom.get("type")
        coords = geom.get("coordinates", [])
        if not coords:
            continue

        if gtype == "MultiPolygon":
            rings = coords[0][0]
        elif gtype == "Polygon":
            rings = coords[0]
        elif gtype == "MultiLineString":
            rings = coords[0]
        else:
            raise ValueError("unsupported GeoJSON geometry type: {}".format(gtype))

        ring = [[float(pt[1]), float(pt[0])] for pt in rings]  # (lon, lat) -> (lat, lon)
        if not polygon:
            polygon = ring
        else:
            obstacles.append(ring)

    return polygon, obstacles, data.get("name", os.path.basename(path))


def load_waypoints_txt(path: str) -> List[List[float]]:
    """Read a ``TurnWPs.txt`` route file into ``[[lat, lon], ...]``."""
    out: List[List[float]] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                continue
            out.append([float(parts[0]), float(parts[1])])
    return out


# --------------------------------------------------------------------------- #
# Bridge
# --------------------------------------------------------------------------- #
class GeoBridge:
    """Bidirectional conversion between WGS84 mission data and the NED raster.

    Parameters
    ----------
    polygon_wgs84:
        ``[[lat, lon], ...]``.  Its first vertex becomes the NED origin, exactly
        as in the upstream :class:`ConvCoords` implementation.
    obstacles_wgs84:
        Optional list of obstacle rings in the same convention.
    """

    def __init__(
        self,
        polygon_wgs84: Sequence[Sequence[float]],
        obstacles_wgs84: Optional[Sequence[Sequence[Sequence[float]]]] = None,
    ):
        from handleGeo.ConvCoords import ConvCoords

        self.polygon_wgs84 = [list(map(float, p)) for p in polygon_wgs84]
        self.obstacles_wgs84 = [
            [list(map(float, p)) for p in ring] for ring in (obstacles_wgs84 or [])
        ]
        self._conv = ConvCoords(self.polygon_wgs84, self.obstacles_wgs84)

        self.polygon_ned = np.asarray(
            self._conv.convWGS84ToNED(self.polygon_wgs84), dtype=float
        )
        self.obstacles_ned = [
            np.asarray(self._conv.convWGS84ToNED(ring), dtype=float)
            for ring in self.obstacles_wgs84
            if len(ring) >= 3
        ]
        self.origin_wgs84 = tuple(self.polygon_wgs84[0])

    # -- conversions ------------------------------------------------------- #
    def wgs84_to_ned(self, points_wgs84: Sequence[Sequence[float]]) -> np.ndarray:
        """``[[lat, lon], ...]`` -> ``(N, 2)`` array of ``(north, east)`` metres."""
        return np.asarray(self._conv.convWGS84ToNED(list(points_wgs84)), dtype=float)

    def ned_to_wgs84(self, points_ned: Sequence[Sequence[float]]) -> List[List[float]]:
        """``[(north, east), ...]`` -> ``[[lat, lon], ...]``."""
        nested = [[[float(p[0]), float(p[1])] for p in points_ned]]
        return self._conv.NEDToWGS84(nested)[0]

    def poses_to_wgs84(self, poses: Sequence[Pose]) -> List[List[float]]:
        return self.ned_to_wgs84([(p.x, p.y) for p in poses])

    def route_to_poses(self, route_ned: Sequence[Sequence[float]]) -> List[Pose]:
        """Build poses with a heading that follows the route direction."""
        poses: List[Pose] = []
        pts = [tuple(map(float, p)) for p in route_ned]
        for k, (x, y) in enumerate(pts):
            if k + 1 < len(pts):
                dx = pts[k + 1][0] - x
                dy = pts[k + 1][1] - y
            elif k > 0:
                dx = x - pts[k - 1][0]
                dy = y - pts[k - 1][1]
            else:
                dx, dy = 1.0, 0.0
            yaw = math.degrees(math.atan2(dy, dx)) if (dx or dy) else 0.0
            poses.append(Pose(x, y, yaw))
        return poses

    # -- grid construction -------------------------------------------------- #
    def build_grid(self, resolution: float = 2.5, margin: float = 0.0) -> CoverageGrid:
        """Rasterise polygon and obstacles into a :class:`CoverageGrid`."""
        cfg = GridConfig.from_polygon(
            self.polygon_ned, resolution=resolution, margin=margin
        )
        grid = CoverageGrid(cfg)
        grid.rasterize_polygon(self.polygon_ned)
        if self.obstacles_ned:
            grid.rasterize_obstacles(self.obstacles_ned)
        return grid

    def reference_route(
        self,
        lane_spacing: float,
        lane_axis: str = "north",
    ) -> List[Pose]:
        """Lawnmower route over the operational polygon, in NED.

        ``lane_spacing`` is the distance between adjacent lanes; for a coverage
        split at ``side_lap`` overlap it is ``ground_width * (1 - side_lap)``,
        which mirrors the ``scanDist`` computation in ``main.py``.
        """
        from ap_cpp.planner import RollingHorizonPlanner

        return RollingHorizonPlanner.boustrophedon_route(
            self.polygon_ned, lane_spacing, altitude_axis=lane_axis
        )

    def describe(self) -> dict:
        return {
            "origin_wgs84": self.origin_wgs84,
            "polygon_vertices": len(self.polygon_wgs84),
            "obstacles": len(self.obstacles_ned),
            "polygon_ned_bounds": {
                "x_min": float(self.polygon_ned[:, 0].min()),
                "x_max": float(self.polygon_ned[:, 0].max()),
                "y_min": float(self.polygon_ned[:, 1].min()),
                "y_max": float(self.polygon_ned[:, 1].max()),
            },
        }
