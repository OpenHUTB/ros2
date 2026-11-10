"""Pure-Python gimbal projection, loss state machine, and metrics."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

Vec3 = tuple[float, float, float]


def vsub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vnorm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value <= -180.0:
        value += 360.0
    return value


def bearing_from_position(position: Vec3, target: Vec3) -> tuple[float, float]:
    delta = vsub(target, position)
    horizontal = math.hypot(delta[0], delta[1])
    if horizontal < 1e-9 and abs(delta[2]) < 1e-9:
        raise ValueError("target and camera positions coincide")
    yaw = math.degrees(math.atan2(delta[1], delta[0]))
    pitch = math.degrees(math.atan2(delta[2], horizontal))
    return wrap_deg(yaw), pitch


def boresight_direction(yaw_deg: float, pitch_deg: float) -> Vec3:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    return (
        math.cos(pitch) * math.cos(yaw),
        math.cos(pitch) * math.sin(yaw),
        math.sin(pitch),
    )


def line_of_sight_error_deg(camera: Vec3, target: Vec3, yaw_deg: float, pitch_deg: float) -> float:
    delta = vsub(target, camera)
    norm = vnorm(delta)
    if norm < 1e-9:
        raise ValueError("target and camera positions coincide")
    los = (delta[0] / norm, delta[1] / norm, delta[2] / norm)
    boresight = boresight_direction(yaw_deg, pitch_deg)
    dot = clamp(sum(a * b for a, b in zip(los, boresight)), -1.0, 1.0)
    return math.degrees(math.acos(dot))


@dataclass(frozen=True)
class GimbalConfig:
    uav_position: Vec3 = (0.0, 0.0, 20.0)
    width: int = 640
    height: int = 480
    hfov_deg: float = 60.0
    vfov_deg: float = 45.0
    max_yaw_rate_deg_s: float = 55.0
    max_pitch_rate_deg_s: float = 40.0
    min_pitch_deg: float = -85.0
    max_pitch_deg: float = 5.0
    initial_yaw_deg: float = 0.0
    initial_pitch_deg: float = 0.0
    loss_frames: int = 3
    confidence_threshold: float = 0.5

    def __post_init__(self) -> None:
        for name in ("width", "height", "hfov_deg", "vfov_deg", "loss_frames"):
            value = getattr(self, name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.hfov_deg >= 180 or self.vfov_deg >= 180:
            raise ValueError("FOV must be smaller than 180 degrees")
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")


@dataclass(frozen=True)
class TargetSample:
    time_s: float
    position: Vec3
    visible: bool


class TargetScenario:
    """Deterministic target trajectory with occlusion and one abrupt relocation."""

    def __init__(self) -> None:
        self.relocation_time_s = 18.0
        self.occlusions = ((8.0, 12.0), (32.0, 36.0))

    def sample(self, time_s: float) -> TargetSample:
        if time_s < self.relocation_time_s:
            x = 30.0 + 5.0 * math.sin(0.12 * time_s)
            y = 10.0 * math.sin(0.25 * time_s)
        else:
            x = 34.0 + 6.0 * math.sin(0.10 * (time_s - self.relocation_time_s))
            y = 20.0 + 8.0 * math.sin(0.22 * (time_s - self.relocation_time_s))
        z = 1.5 + 0.5 * math.sin(0.31 * time_s)
        visible = not any(start <= time_s < end for start, end in self.occlusions)
        return TargetSample(time_s, (x, y, z), visible)


@dataclass(frozen=True)
class Observation:
    time_s: float
    state: str
    detected: bool
    visible: bool
    yaw_deg: float
    pitch_deg: float
    target_yaw_deg: float
    target_pitch_deg: float
    yaw_error_deg: float
    pitch_error_deg: float
    los_error_deg: float
    pixel_u: float | None
    pixel_v: float | None
    in_frame: bool
    confidence: float
    reacquisition_time_s: float | None


class GimbalController:
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    LOST = "LOST"
    REACQUIRE = "REACQUIRE"

    def __init__(self, config: GimbalConfig | None = None) -> None:
        self.config = config or GimbalConfig()
        self.yaw_deg = self.config.initial_yaw_deg
        self.pitch_deg = self.config.initial_pitch_deg
        self.state = self.SEARCH
        self.loss_counter = 0
        self.loss_started_s: float | None = None
        self.search_elapsed_s = 0.0
        self.last_bearing = (0.0, -35.0)
        self.last_reacquisition_s: float | None = None

    def _detect(self, target: Vec3, visible: bool) -> tuple[bool, float, float, float, float, float]:
        target_yaw, target_pitch = bearing_from_position(self.config.uav_position, target)
        yaw_error = wrap_deg(target_yaw - self.yaw_deg)
        pitch_error = target_pitch - self.pitch_deg
        in_frame = (
            visible
            and abs(yaw_error) <= self.config.hfov_deg / 2.0
            and abs(pitch_error) <= self.config.vfov_deg / 2.0
        )
        confidence = 0.0
        if in_frame:
            normalized = math.hypot(yaw_error / (self.config.hfov_deg / 2.0), pitch_error / (self.config.vfov_deg / 2.0))
            confidence = clamp(1.0 - 0.35 * normalized, 0.0, 1.0)
        return in_frame, confidence, target_yaw, target_pitch, yaw_error, pitch_error

    def _set_heading(self, yaw: float, pitch: float, dt_s: float) -> None:
        max_yaw = self.config.max_yaw_rate_deg_s * dt_s
        max_pitch = self.config.max_pitch_rate_deg_s * dt_s
        yaw_delta = clamp(wrap_deg(yaw - self.yaw_deg), -max_yaw, max_yaw)
        pitch_delta = clamp(pitch - self.pitch_deg, -max_pitch, max_pitch)
        self.yaw_deg = wrap_deg(self.yaw_deg + yaw_delta)
        self.pitch_deg = clamp(
            self.pitch_deg + pitch_delta,
            self.config.min_pitch_deg,
            self.config.max_pitch_deg,
        )

    def step(self, sample: TargetSample, dt_s: float) -> Observation:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        detected, confidence, target_yaw, target_pitch, yaw_error, pitch_error = self._detect(
            sample.position, sample.visible
        )

        if detected and confidence >= self.config.confidence_threshold:
            if self.state != self.TRACK:
                if self.loss_started_s is not None:
                    self.last_reacquisition_s = sample.time_s - self.loss_started_s
                    self.loss_started_s = None
                self.state = self.TRACK
            self.loss_counter = 0
            self.last_bearing = (target_yaw, target_pitch)
            desired_yaw, desired_pitch = target_yaw, target_pitch
        else:
            if self.state == self.TRACK:
                self.loss_counter += 1
                if self.loss_counter >= self.config.loss_frames:
                    self.state = self.LOST
                    if self.loss_started_s is None:
                        self.loss_started_s = sample.time_s
            if self.state in (self.SEARCH, self.LOST, self.REACQUIRE):
                self.search_elapsed_s += dt_s
                desired_yaw = wrap_deg(
                    self.last_bearing[0] + 62.0 * math.sin(0.75 * self.search_elapsed_s)
                )
                desired_pitch = clamp(
                    self.last_bearing[1] + 16.0 * math.sin(1.05 * self.search_elapsed_s),
                    self.config.min_pitch_deg,
                    self.config.max_pitch_deg,
                )
            else:
                desired_yaw, desired_pitch = self.yaw_deg, self.pitch_deg

        self._set_heading(desired_yaw, desired_pitch, dt_s)
        detected_after, confidence_after, target_yaw_after, target_pitch_after, yaw_error_after, pitch_error_after = self._detect(
            sample.position, sample.visible
        )
        los_error = line_of_sight_error_deg(
            self.config.uav_position,
            sample.position,
            self.yaw_deg,
            self.pitch_deg,
        )
        if detected_after:
            pixel_u = (
                math.tan(math.radians(yaw_error_after))
                / math.tan(math.radians(self.config.hfov_deg / 2.0))
                * self.config.width
                / 2.0
            )
            pixel_v = (
                -math.tan(math.radians(pitch_error_after))
                / math.tan(math.radians(self.config.vfov_deg / 2.0))
                * self.config.height
                / 2.0
            )
        else:
            pixel_u = None
            pixel_v = None
        return Observation(
            time_s=sample.time_s,
            state=self.state,
            detected=detected_after,
            visible=sample.visible,
            yaw_deg=self.yaw_deg,
            pitch_deg=self.pitch_deg,
            target_yaw_deg=target_yaw_after,
            target_pitch_deg=target_pitch_after,
            yaw_error_deg=yaw_error_after,
            pitch_error_deg=pitch_error_after,
            los_error_deg=los_error,
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            in_frame=detected_after,
            confidence=confidence_after,
            reacquisition_time_s=self.last_reacquisition_s,
        )


@dataclass
class Metrics:
    total_frames: int = 0
    tracking_frames: int = 0
    detection_frames: int = 0
    loss_events: int = 0
    reacquisition_events: int = 0
    reacquisition_times_s: list[float] = field(default_factory=list)
    los_errors_deg: list[float] = field(default_factory=list)
    pixel_errors_px: list[float] = field(default_factory=list)
    max_los_error_deg: float = 0.0
    state_counts: dict[str, int] = field(default_factory=dict)
    _last_state: str | None = None

    def update(self, observation: Observation) -> None:
        self.total_frames += 1
        self.state_counts[observation.state] = self.state_counts.get(observation.state, 0) + 1
        if observation.detected:
            self.detection_frames += 1
        if observation.state == GimbalController.TRACK and observation.detected:
            self.tracking_frames += 1
            self.los_errors_deg.append(observation.los_error_deg)
            if observation.pixel_u is not None and observation.pixel_v is not None:
                self.pixel_errors_px.append(math.hypot(observation.pixel_u, observation.pixel_v))
            self.max_los_error_deg = max(self.max_los_error_deg, observation.los_error_deg)
        if self._last_state != observation.state:
            if observation.state == GimbalController.LOST:
                self.loss_events += 1
            if observation.reacquisition_time_s is not None and observation.state == GimbalController.TRACK:
                if not self.reacquisition_times_s or observation.reacquisition_time_s != self.reacquisition_times_s[-1]:
                    self.reacquisition_events += 1
                    self.reacquisition_times_s.append(observation.reacquisition_time_s)
        self._last_state = observation.state

    def summary(self) -> dict:
        def rms(values: Iterable[float]) -> float:
            values = list(values)
            return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0

        return {
            "total_frames": self.total_frames,
            "tracking_frames": self.tracking_frames,
            "detection_frames": self.detection_frames,
            "track_ratio": self.tracking_frames / max(1, self.total_frames),
            "loss_events": self.loss_events,
            "reacquisition_events": self.reacquisition_events,
            "reacquisition_times_s": list(self.reacquisition_times_s),
            "los_rms_deg": rms(self.los_errors_deg),
            "pixel_rms_px": rms(self.pixel_errors_px),
            "max_los_error_deg": self.max_los_error_deg,
            "state_counts": dict(self.state_counts),
        }
