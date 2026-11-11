"""Deterministic target trajectories for complete simulation episodes."""

from dataclasses import dataclass
from math import isfinite, pi, sin


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
        values = (
            self.duration_s,
            self.initial_angle_rad,
            self.target_angle_rad,
            self.amplitude_rad,
            self.offset_rad,
            self.frequency_hz,
            self.step_time_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("trajectory values must be finite")
        if self.kind not in {"step", "sine"}:
            raise ValueError("trajectory kind must be 'step' or 'sine'")
        if self.duration_s <= 0.0 or self.frequency_hz <= 0.0:
            raise ValueError("duration and frequency must be positive")

    def angle_bounds_rad(self) -> tuple[float, float]:
        if self.kind == "step":
            return min(self.initial_angle_rad, self.target_angle_rad), max(
                self.initial_angle_rad, self.target_angle_rad
            )
        amplitude = abs(self.amplitude_rad)
        return self.offset_rad - amplitude, self.offset_rad + amplitude

    def validate_joint_range(self, lower: float, upper: float) -> None:
        trajectory_lower, trajectory_upper = self.angle_bounds_rad()
        if trajectory_lower < lower or trajectory_upper > upper:
            raise ValueError(
                f"trajectory angle range [{trajectory_lower}, {trajectory_upper}] "
                f"is outside joint range [{lower}, {upper}]"
            )

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
