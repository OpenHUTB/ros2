#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""比例视觉伺服：门中心标签 → 航向系速度（m/s）。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .geometry import CAM_OFFSET_BODY

__all__ = ["ServoConfig", "GateServo", "is_plausible_label"]


@dataclass
class ServoConfig:
    k_x: float = 1.5
    look_ahead: float = 1.0
    v_min: float = 0.5
    v_max: float = 1.8
    k_y: float = 1.2
    k_z: float = 1.2
    v_z_max: float = 0.9
    valid_frac_min: float = 0.5


def is_plausible_label(label: np.ndarray) -> bool:
    """有限且前向距离在合理范围内。"""
    a = np.asarray(label, dtype=float).reshape(-1)
    if a.size != 3 or not np.all(np.isfinite(a)):
        return False
    return -2.0 <= float(a[0]) <= 10.0


class GateServo:
    """门中心标签 → 航向系速度；oracle 与 MLP 共用。"""

    def __init__(self, cfg: Optional[ServoConfig] = None):
        self.cfg = cfg or ServoConfig()

    def command(self, label_cam: np.ndarray,
                cam_offset_world: Optional[np.ndarray] = None,
                valid_frac: float = 1.0,
                valid_prediction: bool = True) -> np.ndarray:
        cfg = self.cfg
        if not valid_prediction or not is_plausible_label(label_cam):
            return np.zeros(3, dtype=np.float32)
        offset = CAM_OFFSET_BODY if cam_offset_world is None \
            else np.asarray(cam_offset_world, float).reshape(3)
        target = np.asarray(label_cam, float).reshape(3) + offset
        if target[0] <= 0.0:
            return np.zeros(3, dtype=np.float32)
        vx = np.clip(cfg.k_x * (target[0] - cfg.look_ahead), cfg.v_min, cfg.v_max)
        vy = np.clip(cfg.k_y * target[1], -cfg.v_max, cfg.v_max)
        vz = np.clip(cfg.k_z * target[2], -cfg.v_z_max, cfg.v_z_max)
        scale = float(np.clip(valid_frac / max(cfg.valid_frac_min, 1e-6), 0.0, 1.0))
        return (np.array([vx, vy, vz], np.float32) * scale).astype(np.float32)
