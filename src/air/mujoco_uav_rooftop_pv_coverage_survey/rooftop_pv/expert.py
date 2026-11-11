"""Deterministic flight-line expert / 确定性航迹专家控制器。"""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi] / 将角度归一化到 [-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ExpertConfig:
    """Expert gains and limits / 专家控制增益与限幅。"""

    cruise_speed: float = 2.0
    min_speed: float = 0.25
    max_yaw_rate: float = 1.2
    cross_track_gain: float = 0.35
    heading_gain: float = 1.4
    turn_slow_distance: float = 2.5

    def validate(self) -> None:
        values = {
            "cruise_speed": self.cruise_speed,
            "min_speed": self.min_speed,
            "max_yaw_rate": self.max_yaw_rate,
            "cross_track_gain": self.cross_track_gain,
            "heading_gain": self.heading_gain,
            "turn_slow_distance": self.turn_slow_distance,
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in values.values()):
            raise ValueError("expert parameters must be finite and positive / 专家参数必须为有限正数")
        if self.min_speed > self.cruise_speed:
            raise ValueError("min_speed cannot exceed cruise_speed / 最低速度不能超过巡航速度")


@dataclass(frozen=True)
class ExpertInput:
    """Neural-controller training features / 神经控制器训练特征。"""

    cross_track_error: float
    heading_error: float
    distance_to_turn: float


@dataclass(frozen=True)
class ExpertCommand:
    """Forward speed and yaw-rate targets / 前进速度与偏航角速度目标。"""

    forward_speed: float
    yaw_rate: float


class FlightLineExpert:
    """Map path errors to bounded deterministic commands / 将航迹误差映射为确定性限幅指令。"""

    def __init__(self, config: ExpertConfig | None = None) -> None:
        self.config = config or ExpertConfig()
        self.config.validate()

    def command(self, state: ExpertInput) -> ExpertCommand:
        values = (state.cross_track_error, state.heading_error, state.distance_to_turn)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("expert inputs must be finite / 专家输入必须为有限数")
        if state.distance_to_turn < 0.0:
            raise ValueError("distance_to_turn cannot be negative / 距转弯点距离不能为负")

        heading_error = wrap_angle(state.heading_error)
        yaw_rate = -(
            self.config.cross_track_gain * state.cross_track_error
            + self.config.heading_gain * heading_error
        )
        yaw_rate = max(-self.config.max_yaw_rate, min(self.config.max_yaw_rate, yaw_rate))

        turn_factor = min(1.0, max(0.0, state.distance_to_turn / self.config.turn_slow_distance))
        alignment_factor = max(0.25, math.cos(heading_error))
        desired = self.config.cruise_speed * (0.35 + 0.65 * turn_factor) * alignment_factor
        speed = max(self.config.min_speed, min(self.config.cruise_speed, desired))
        return ExpertCommand(forward_speed=speed, yaw_rate=yaw_rate)
