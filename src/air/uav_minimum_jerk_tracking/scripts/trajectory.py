#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多段五次最小加加速度轨迹与线性参考。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    elapsed: float
    total_duration: float
    segment_index: int


@dataclass(frozen=True)
class _Segment:
    start: np.ndarray
    end: np.ndarray
    duration: float
    mode: str


def minimum_jerk_duration(
    distance,
    max_speed,
    max_acceleration,
    minimum_duration=1.0,
):
    """满足峰值速度/加速度约束的最短五次轨迹时长."""
    distance = max(float(distance), 0.0)
    if distance <= 1e-12:
        return max(float(minimum_duration), 1e-3)
    velocity_duration = 1.875 * distance / max(float(max_speed), 1e-6)
    acceleration_duration = math.sqrt(
        5.773502691896258 * distance / max(float(max_acceleration), 1e-6)
    )
    return max(float(minimum_duration), velocity_duration, acceleration_duration)


def minimum_jerk_state(start, end, duration, elapsed):
    """返回五次最小加加速度段的位置、速度、加速度和加加速度."""
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    duration = max(float(duration), 1e-6)
    u = float(np.clip(float(elapsed) / duration, 0.0, 1.0))
    delta = end - start

    position = start + delta * (10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5)
    velocity = delta * (
        (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
    )
    acceleration = delta * (
        (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / duration**2
    )
    jerk = delta * (
        (60.0 - 360.0 * u + 360.0 * u**2) / duration**3
    )
    return position, velocity, acceleration, jerk


def linear_state(start, end, duration, elapsed):
    """线性位置参考；速度在段边界不连续."""
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    duration = max(float(duration), 1e-6)
    u = float(np.clip(float(elapsed) / duration, 0.0, 1.0))
    delta = end - start
    position = start + delta * u
    velocity = delta / duration
    return (
        position,
        velocity,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
    )


class MinimumJerkTrajectory:
    """按顺序经过绝对世界坐标航点的轨迹."""

    def __init__(self, start, waypoints, params, mode="minimum_jerk"):
        self.mode = str(mode)
        if self.mode not in ("minimum_jerk", "linear"):
            raise ValueError("mode 必须是 minimum_jerk 或 linear")

        points = [np.asarray(start, dtype=np.float64)]
        for waypoint in waypoints:
            point = np.asarray(waypoint, dtype=np.float64).reshape(3)
            if np.linalg.norm(point - points[-1]) > 1e-9:
                points.append(point)
        if len(points) == 1:
            raise ValueError("轨迹至少需要一个与起点不同的航点")

        self.segments = []
        for begin, end in zip(points[:-1], points[1:]):
            delta = end - begin
            horizontal_distance = float(np.linalg.norm(delta[:2]))
            vertical_distance = abs(float(delta[2]))
            duration_values = []
            if horizontal_distance > 1e-9:
                duration_values.append(
                    minimum_jerk_duration(
                        horizontal_distance,
                        params["max_speed_horizontal"],
                        params["max_acceleration_horizontal"],
                        params["min_segment_duration"],
                    )
                    if self.mode == "minimum_jerk"
                    else max(
                        horizontal_distance / params["max_speed_horizontal"],
                        params["min_segment_duration"],
                    )
                )
            if vertical_distance > 1e-9:
                duration_values.append(
                    minimum_jerk_duration(
                        vertical_distance,
                        params["max_speed_vertical"],
                        params["max_acceleration_vertical"],
                        params["min_segment_duration"],
                    )
                    if self.mode == "minimum_jerk"
                    else max(
                        vertical_distance / params["max_speed_vertical"],
                        params["min_segment_duration"],
                    )
                )
            duration = max(duration_values or [params["min_segment_duration"]])
            self.segments.append(_Segment(begin, end, duration, self.mode))

        self._starts = np.cumsum(
            [0.0] + [segment.duration for segment in self.segments[:-1]]
        )
        self.total_duration = float(
            sum(segment.duration for segment in self.segments)
        )

    def sample(self, elapsed):
        elapsed = float(np.clip(float(elapsed), 0.0, self.total_duration))
        index = int(np.searchsorted(self._starts, elapsed, side="right") - 1)
        index = int(np.clip(index, 0, len(self.segments) - 1))
        segment = self.segments[index]
        local_time = elapsed - self._starts[index]
        if self.mode == "minimum_jerk":
            state = minimum_jerk_state(
                segment.start, segment.end, segment.duration, local_time
            )
        else:
            state = linear_state(
                segment.start, segment.end, segment.duration, local_time
            )
        return TrajectorySample(
            position=state[0],
            velocity=state[1],
            acceleration=state[2],
            jerk=state[3],
            elapsed=elapsed,
            total_duration=self.total_duration,
            segment_index=index,
        )

    def sampled_path(self, dt=0.1):
        count = max(2, int(math.ceil(self.total_duration / float(dt))) + 1)
        return [self.sample(i * float(dt)) for i in range(count)]
