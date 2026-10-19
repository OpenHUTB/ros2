"""Deterministic path-centering expert / 确定性路径居中专家。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class SteeringLabel(str, Enum):
    """Discrete supervision labels / 离散监督标签。"""

    LEFT = "LEFT"
    STRAIGHT = "STRAIGHT"
    RIGHT = "RIGHT"
    STOP = "STOP"


@dataclass(frozen=True)
class ExpertDecision:
    """Expert output for one frame / 单帧专家输出。"""

    steering: float
    label: SteeringLabel
    lateral_error_m: float
    heading_error_rad: float


class PathCenterExpert:
    """Proportional expert using path-center and heading errors / 使用路径中心及航向误差的比例专家。"""

    def __init__(
        self,
        path_half_width_m: float,
        steering_threshold: float,
        stop_margin_m: float,
        path_end_y_m: float,
        lateral_gain: float = 0.85,
        heading_gain: float = 0.65,
    ) -> None:
        if path_half_width_m <= 0.0:
            raise ValueError("path_half_width_m must be positive / 路径半宽必须为正数")
        if not 0.0 < steering_threshold < 1.0:
            raise ValueError("steering_threshold must be in (0, 1) / 转向阈值必须位于 (0, 1)")
        if not 0.0 <= stop_margin_m < path_half_width_m:
            raise ValueError("stop_margin_m is invalid / 停止边界余量无效")
        self.path_half_width_m = path_half_width_m
        self.steering_threshold = steering_threshold
        self.stop_margin_m = stop_margin_m
        self.path_end_y_m = path_end_y_m
        self.lateral_gain = lateral_gain
        self.heading_gain = heading_gain

    def decide(self, x_m: float, y_m: float, yaw_radians: float) -> ExpertDecision:
        """Return steering and label for the current pose / 根据当前姿态返回转向量与标签。"""
        lateral_error = -float(x_m)
        heading_error = -float(yaw_radians)
        safe_limit = self.path_half_width_m - self.stop_margin_m
        if abs(x_m) >= safe_limit or y_m >= self.path_end_y_m:
            return ExpertDecision(0.0, SteeringLabel.STOP, lateral_error, heading_error)

        steering = float(
            np.clip(
                self.lateral_gain * lateral_error + self.heading_gain * heading_error,
                -1.0,
                1.0,
            )
        )
        if steering < -self.steering_threshold:
            label = SteeringLabel.LEFT
        elif steering > self.steering_threshold:
            label = SteeringLabel.RIGHT
        else:
            label = SteeringLabel.STRAIGHT
        return ExpertDecision(steering, label, lateral_error, heading_error)

