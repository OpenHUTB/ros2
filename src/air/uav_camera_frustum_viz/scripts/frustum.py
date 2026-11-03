"""Camera frustum geometry, pure numpy (no ROS import).

Optical frame: x right, y down, z forward.
Body FLU: x forward, y left, z up, so optical -> body = [z, -x, -y].
"""
from __future__ import annotations

import math

import numpy as np


def frustum_corners(width, height, fx, fy, cx, cy, near=0.5, far=8.0):
    """Return 8 corners in the optical frame: near 0-3, far 4-7."""
    corners = []
    for z in (float(near), float(far)):
        for u, v in ((0.0, 0.0), (width, 0.0), (0.0, height), (width, height)):
            corners.append([(u - cx) * z / fx, (v - cy) * z / fy, z])
    return np.asarray(corners, dtype=np.float64)


def optical_to_body(points):
    pts = np.asarray(points, dtype=np.float64)
    return np.stack([pts[:, 2], -pts[:, 0], -pts[:, 1]], axis=1)


def quat_to_matrix(q_xyzw):
    x, y, z, w = [float(v) for v in q_xyzw]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def world_corners(position, orientation_xyzw, width, height,
                  fx, fy, cx, cy, near=0.5, far=8.0):
    """Camera corners in world ENU."""
    optical = frustum_corners(width, height, fx, fy, cx, cy, near, far)
    body = optical_to_body(optical)
    rot = quat_to_matrix(orientation_xyzw)
    return body @ rot.T + np.asarray(position, dtype=np.float64)


EDGES = ((0, 1), (1, 3), (3, 2), (2, 0),
         (4, 5), (5, 7), (7, 6), (6, 4),
         (0, 4), (1, 5), (2, 6), (3, 7))


def edge_points(corners):
    """Flat list of point pairs for a LINE_LIST marker."""
    pts = []
    for a, b in EDGES:
        pts.extend([corners[a], corners[b]])
    return np.asarray(pts, dtype=np.float64)
