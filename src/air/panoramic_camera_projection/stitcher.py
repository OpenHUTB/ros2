"""Projection utilities for six-face panoramic camera images."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import cv2
import numpy as np


FACE_NAMES = ("front", "right", "back", "left", "up", "down")


def face_for_direction(x: float, y: float, z: float) -> Tuple[str, float, float]:
    """Return the cube face and centered coordinates in [-1, 1]."""
    ax, ay, az = abs(x), abs(y), abs(z)
    if ax >= ay and ax >= az:
        if x >= 0:
            return "front", y / ax, -z / ax
        return "back", -y / ax, -z / ax
    if ay >= ax and ay >= az:
        if y >= 0:
            return "right", -x / ay, -z / ay
        return "left", x / ay, -z / ay
    if z >= 0:
        return "up", x / az, y / az
    return "down", x / az, -y / az


def stitch_cubemap(
    faces: Dict[str, np.ndarray],
    output_size: Tuple[int, int] = (512, 1024),
) -> np.ndarray:
    """Convert six equally sized cube faces to an equirectangular image."""
    missing = set(FACE_NAMES) - set(faces)
    if missing:
        raise ValueError(f"missing cube faces: {sorted(missing)}")

    height, width = output_size
    face_size = min(faces[name].shape[0] for name in FACE_NAMES)
    if any(faces[name].shape[:2] != (face_size, face_size) for name in FACE_NAMES):
        raise ValueError("all cube faces must have the same square resolution")

    longitude = np.linspace(-math.pi, math.pi, width, endpoint=False)
    latitude = np.linspace(math.pi / 2.0, -math.pi / 2.0, height)
    lon, lat = np.meshgrid(longitude, latitude)
    cos_lat = np.cos(lat)
    dx = cos_lat * np.cos(lon)
    dy = cos_lat * np.sin(lon)
    dz = np.sin(lat)

    output = np.zeros((height, width, faces["front"].shape[2]), dtype=np.uint8)
    max_pixel = face_size - 1
    for row in range(height):
        for col in range(width):
            name, u, v = face_for_direction(dx[row, col], dy[row, col], dz[row, col])
            px = int(round((u + 1.0) * 0.5 * max_pixel))
            py = int(round((v + 1.0) * 0.5 * max_pixel))
            px = min(max(px, 0), max_pixel)
            py = min(max(py, 0), max_pixel)
            output[row, col] = faces[name][py, px]
    return output


def create_demo_cubemap(size: int = 256) -> Dict[str, np.ndarray]:
    """Create a labeled six-face cubemap for an offline visual check."""
    colors = {
        "front": (40, 60, 220),
        "right": (40, 180, 60),
        "back": (220, 80, 40),
        "left": (180, 60, 180),
        "up": (40, 200, 220),
        "down": (200, 200, 40),
    }
    faces: Dict[str, np.ndarray] = {}
    for name in FACE_NAMES:
        image = np.empty((size, size, 3), dtype=np.uint8)
        image[:] = colors[name]
        cv2.putText(
            image,
            name.upper(),
            (size // 20, size // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            size / 240.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        faces[name] = image
    return faces
