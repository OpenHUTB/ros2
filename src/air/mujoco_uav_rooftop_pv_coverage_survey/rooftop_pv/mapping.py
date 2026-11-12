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
    sequence_frames: dict[str, set[int]] = {}
    expected_frames: dict[str, int] = {}
    roof_width = float(rows[0]["roof_width_m"]) if rows else 1.0
    roof_depth = float(rows[0]["roof_depth_m"]) if rows else 1.0
    if roof_width <= 0 or roof_depth <= 0:
        raise ValueError("roof dimensions in manifest must be positive")
    for row in rows:
        sequence_id = row["sequence_id"]
        sequence_frames.setdefault(sequence_id, set()).add(int(row["frame_index"]))
        expected_frames[sequence_id] = int(row["sequence_frame_count"])
        if float(row["roof_width_m"]) != roof_width or float(row["roof_depth_m"]) != roof_depth:
            raise ValueError("roof dimensions must be consistent across the manifest")
        image = np.asarray(Image.open(dataset / row["mask_path"]).convert("L")) > 0
        ys, xs = np.nonzero(image)
        if len(xs) == 0:
            continue
        pose = row["pose"]
        # Nadir camera projection: normalized pixel offsets become local ENU offsets.
        local_x = (xs / max(image.shape[1] - 1, 1) - 0.5) * roof_width
        local_y = (ys / max(image.shape[0] - 1, 1) - 0.5) * roof_depth
        c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        world_x = pose["x"] + c * local_x - s * local_y
        world_y = pose["y"] + s * local_x + c * local_y
        gx = np.clip(((world_x + roof_width / 2) / roof_width * (grid_size - 1)).astype(int), 0, grid_size - 1)
        gy = np.clip(((world_y + roof_depth / 2) / roof_depth * (grid_size - 1)).astype(int), 0, grid_size - 1)
        occupancy[gy, gx] = 1
    total_expected = sum(expected_frames.values())
    completed_frames = sum(len(indices) for indices in sequence_frames.values())
    return {
        "grid_size": grid_size,
        "mapped_cells": int(occupancy.sum()),
        "map_coverage": float(occupancy.mean()),
        "sequences": len(sequence_frames),
        "route_frames": len(rows),
        "route_replay_progress": float(completed_frames / total_expected) if total_expected else 0.0,
        "sensor": "downward_rgb_pv_mask",
        "mapping_mode": "synchronized_pose_fusion",
    }

