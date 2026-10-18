"""Deterministic target trajectories for complete simulation episodes."""

from dataclasses import dataclass
from math import pi, sin


@dataclass(frozen=True)
class TrajectorySpec:
    kind: str
    duration_s: float
    initial_angle_rad: float = 0.35
    target_angle_rad: float = 1.2
    amplitude_rad: float = 0.45
    offset_rad: float = 0.8
    frequency_hz: float = 0.25
    step_time_s: float = 0.5

    def __post_init__(self) -> None:
        if self.kind not in {"step", "sine"}:
            raise ValueError("trajectory kind must be 'step' or 'sine'")
        if self.duration_s <= 0.0 or self.frequency_hz <= 0.0:
            raise ValueError("duration and frequency must be positive")

    def sample(self, time_s: float) -> float:
        if self.kind == "step":
            return (
                self.initial_angle_rad
                if time_s < self.step_time_s
                else self.target_angle_rad
            )
        return self.offset_rad + self.amplitude_rad * sin(
            2.0 * pi * self.frequency_hz * time_s
        )
