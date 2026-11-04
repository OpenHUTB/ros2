#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞行过程最小间距、路径长度与到达状态统计。"""

from __future__ import annotations

import csv
import math
import os
import time

import numpy as np


class FlightMetrics:
    def __init__(self):
        self.started = time.time()
        self.previous_position = None
        self.path_length = 0.0
        self.min_clearance = float("inf")
        self.samples = 0
        self.final_distance = float("inf")
        self.success = False

    def update(self, position, goal, points, command):
        position = np.asarray(position, dtype=np.float64)
        if self.previous_position is not None:
            self.path_length += float(
                np.linalg.norm(position - self.previous_position)
            )
        self.previous_position = position
        self.samples += 1
        self.final_distance = float(
            np.linalg.norm(np.asarray(goal, dtype=np.float64) - position)
        )

        cloud = np.asarray(points, dtype=np.float64)
        if cloud.size:
            near = float(
                np.min(np.linalg.norm(cloud - position, axis=1))
            )
            self.min_clearance = min(self.min_clearance, near)

        command_norm = math.sqrt(
            command.vx * command.vx + command.vy * command.vy + command.vz * command.vz
        )
        if command_norm < 1e-9 and self.final_distance < 1.5:
            self.success = True

    def write(self, path):
        if not path:
            return
        path = os.path.expanduser(path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        row = {
            "duration_s": time.time() - self.started,
            "path_length_m": self.path_length,
            "min_clearance_m": self.min_clearance,
            "samples": self.samples,
            "final_distance_m": self.final_distance,
            "success": int(self.success),
        }
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
