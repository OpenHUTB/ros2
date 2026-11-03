"""Small, dependency-light geometry helpers used by the airspace monitor."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

import numpy as np


EARTH_RADIUS_M = 6_371_000.0


def point_in_polygon(point: Sequence[float], polygon: Sequence[Sequence[float]]) -> bool:
    """Return True when a 2-D point is inside or on the polygon boundary."""
    x, y = float(point[0]), float(point[1])
    inside = False
    vertices = list(polygon)
    if len(vertices) < 3:
        return False

    for index, current in enumerate(vertices):
        nxt = vertices[(index + 1) % len(vertices)]
        x1, y1 = float(current[0]), float(current[1])
        x2, y2 = float(nxt[0]), float(nxt[1])

        if _point_on_segment(x, y, x1, y1, x2, y2):
            return True
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def _point_on_segment(
    x: float, y: float, x1: float, y1: float, x2: float, y2: float
) -> bool:
    cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
    if abs(cross) > 1e-9:
        return False
    return (
        min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9
        and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9
    )


def distance_point_to_segment(
    point: Sequence[float],
    start: Sequence[float],
    end: Sequence[float],
) -> float:
    """Distance from a 2-D point to a finite line segment."""
    p = np.asarray(point[:2], dtype=float)
    a = np.asarray(start[:2], dtype=float)
    b = np.asarray(end[:2], dtype=float)
    ab = b - a
    denom = float(ab @ ab)
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def distance_to_polygon(
    point: Sequence[float], polygon: Sequence[Sequence[float]]
) -> float:
    """Signed-free distance to the polygon outline (0 when inside)."""
    if point_in_polygon(point, polygon):
        return 0.0
    vertices = list(polygon)
    return min(
        distance_point_to_segment(point, vertices[i], vertices[(i + 1) % len(vertices)])
        for i in range(len(vertices))
    )


def polygon_area(polygon: Sequence[Sequence[float]]) -> float:
    """Area of a simple polygon in square metres."""
    vertices = list(polygon)
    if len(vertices) < 3:
        return 0.0
    total = 0.0
    for index, current in enumerate(vertices):
        nxt = vertices[(index + 1) % len(vertices)]
        total += float(current[0]) * float(nxt[1]) - float(nxt[0]) * float(current[1])
    return abs(total) * 0.5


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def latlon_to_local(
    lat: float, lon: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """Equirectangular local ENU conversion for small survey areas."""
    east = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(
        math.radians(origin_lat)
    )
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    return east, north


def local_to_latlon(
    east: float, north: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """Inverse of :func:`latlon_to_local`."""
    lat = origin_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        east / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    return lat, lon


def as_array(values: Iterable[Sequence[float]]) -> np.ndarray:
    """Convert a sequence of 2-D/3-D points to a float array."""
    array = np.asarray(list(values), dtype=float)
    if array.ndim != 2:
        raise ValueError("points must be a 2-D array of shape (N, 2) or (N, 3)")
    return array
