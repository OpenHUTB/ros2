"""Sensor projection, occupancy mapping, and route navigation metrics / 传感器投影、占据栅格与航线导航指标。"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import math

import numpy as np
from PIL import Image


def map_and_navigate(dataset: Path, grid_size: int = 64) -> dict[str, Any]:
    """Fuse PV pixels into a local occupancy grid while replaying each route sequence."""
    rows = [json.loads(line) for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8").splitlines()]
    occupancy = np.zeros((grid_size, grid_size), dtype=np.uint8)
    sequence_progress: dict[str, float] = {}
    for row in rows:
        image = np.asarray(Image.open(dataset / row["mask_path"]).convert("L")) > 0
        ys, xs = np.nonzero(image)
        if len(xs) == 0:
            continue
        pose = row["pose"]
        # Nadir camera projection: normalized pixel offsets become local ENU offsets.
        local_x = (xs / max(image.shape[1] - 1, 1) - 0.5) * 20.0
        local_y = (ys / max(image.shape[0] - 1, 1) - 0.5) * 14.0
        c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        world_x = pose["x"] + c * local_x - s * local_y
        world_y = pose["y"] + s * local_x + c * local_y
        gx = np.clip(((world_x + 10.0) / 20.0 * (grid_size - 1)).astype(int), 0, grid_size - 1)
        gy = np.clip(((world_y + 7.0) / 14.0 * (grid_size - 1)).astype(int), 0, grid_size - 1)
        occupancy[gy, gx] = 1
        sequence_progress[row["sequence_id"]] = sequence_progress.get(row["sequence_id"], 0.0) + 1.0
    total = max(len(rows), 1)
    return {
        "grid_size": grid_size,
        "mapped_cells": int(occupancy.sum()),
        "map_coverage": float(occupancy.mean()),
        "sequences": len(sequence_progress),
        "route_frames": len(rows),
        "navigation_progress": float(sum(sequence_progress.values()) / total),
        "sensor": "downward_rgb_pv_mask",
        "slam_mode": "synchronized_pose_fusion",
    }
