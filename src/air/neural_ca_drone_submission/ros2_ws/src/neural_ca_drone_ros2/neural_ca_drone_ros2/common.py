"""Shared helpers for the ROS 2 nodes."""

from __future__ import annotations

import math
import os
import sys

import numpy as np


def add_repo_to_path() -> str:
    """Locate the repository root (the directory that contains ``ca_drone``)."""
    here = os.path.dirname(os.path.abspath(__file__))
    root = here
    for _ in range(8):
        if os.path.isdir(os.path.join(root, "ca_drone")):
            break
        root = os.path.dirname(root)
    if root not in sys.path:
        sys.path.insert(0, root)
    return root


def decode_depth(msg, depth_w: int, depth_h: int, max_range: float) -> np.ndarray:
    """Convert a ROS depth image to the (48, 64) range image used by the CNNs."""
    h, w = int(msg.height), int(msg.width)
    if msg.encoding == "32FC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)
    elif msg.encoding == "16UC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
    elif msg.encoding in ("8UC1", "mono8"):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w).astype(np.float32) / 255.0 * max_range
    else:
        raise ValueError("unsupported depth encoding: %s" % msg.encoding)
    ys = np.linspace(0, h - 1, depth_h).astype(int)
    xs = np.linspace(0, w - 1, depth_w).astype(int)
    out = arr[np.ix_(ys, xs)]
    out = np.where(np.isfinite(out), out, max_range)
    return np.clip(out, 0.0, max_range).astype(np.float32)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
