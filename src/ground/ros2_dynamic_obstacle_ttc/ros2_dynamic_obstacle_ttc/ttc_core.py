"""Clustering, tracking and time-to-collision for 2-D laser scans.

The module is pure Python/NumPy so it can be tested without ROS or Gazebo.
It is intentionally independent from ``ros2_dwa_avoidance``: this is a
perception/safety layer, not another planner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class Cluster:
    x: float
    y: float
    radius: float
    count: int


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    missed: int = 0
    last_t: float = 0.0
    radius: float = 0.2
    history: List[Tuple[float, float, float]] = field(default_factory=list)


def cluster_scan(ranges: Sequence[float], angle_min: float,
                 angle_increment: float, max_range: float = 6.0,
                 angle_gap: float = 0.12, distance_gap: float = 0.65,
                 min_points: int = 3, max_radius: float = 3.5) -> List[Cluster]:
    """Split a scan into connected obstacle blobs.

    Consecutive valid beams belong to the same blob while the angular step and
    the range jump stay small; invalid beams always split a blob.  Large blobs
    are kept (they may be walls) and later excluded from TTC by their radius.
    """
    ranges_arr = np.asarray(ranges, dtype=float)
    clusters: List[Cluster] = []
    current: List[Tuple[float, float]] = []
    previous = None

    def flush() -> None:
        nonlocal current
        if len(current) >= min_points:
            pts = np.asarray(current, dtype=float)
            cx, cy = pts.mean(axis=0)
            radius = float(np.max(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)))
            if radius <= max_radius:
                clusters.append(Cluster(float(cx), float(cy), radius, len(current)))
        current = []

    for i, r in enumerate(ranges_arr):
        valid = math.isfinite(r) and 0.05 < r <= max_range
        if not valid:
            flush()
            previous = None
            continue
        angle = angle_min + i * angle_increment
        point = (r * math.cos(angle), r * math.sin(angle))
        if previous is not None:
            d_angle = abs(angle - previous[0])
            d_range = abs(r - previous[1])
            if d_angle > angle_gap or d_range > distance_gap:
                flush()
        current.append(point)
        previous = (angle, r)
    flush()
    return clusters


class Tracker:
    """Greedy nearest-neighbour tracker with multi-frame least-squares velocity."""

    def __init__(self, gate: float = 0.9, max_missed: int = 3,
                 history_size: int = 5) -> None:
        self.gate = gate
        self.max_missed = max_missed
        self.history_size = history_size
        self.tracks: List[Track] = []
        self._next_id = 1

    def _fit_velocity(self, track: Track) -> None:
        if len(track.history) < 3:
            if len(track.history) >= 2:
                t0, x0, y0 = track.history[-2]
                t1, x1, y1 = track.history[-1]
                dt = max(1e-3, t1 - t0)
                track.vx, track.vy = (x1 - x0) / dt, (y1 - y0) / dt
            return
        arr = np.asarray(track.history, dtype=float)
        times = arr[:, 0] - arr[0, 0]
        vx = np.polyfit(times, arr[:, 1], 1)[0] if times[-1] > 1e-6 else 0.0
        vy = np.polyfit(times, arr[:, 2], 1)[0] if times[-1] > 1e-6 else 0.0
        track.vx, track.vy = float(vx), float(vy)

    def update(self, clusters: Sequence[Cluster], t: float) -> List[Track]:
        unmatched = list(range(len(clusters)))
        used_tracks = set()
        for track in self.tracks:
            if not unmatched:
                break
            best = min(unmatched, key=lambda i: math.hypot(
                clusters[i].x - track.x, clusters[i].y - track.y))
            distance = math.hypot(clusters[best].x - track.x,
                                  clusters[best].y - track.y)
            if distance <= self.gate:
                cluster = clusters[best]
                track.x, track.y = cluster.x, cluster.y
                track.radius = cluster.radius
                track.hits += 1
                track.missed = 0
                track.last_t = t
                track.history.append((t, cluster.x, cluster.y))
                track.history = track.history[-self.history_size:]
                self._fit_velocity(track)
                used_tracks.add(id(track))
                unmatched.remove(best)

        for track in self.tracks:
            if id(track) not in used_tracks:
                track.missed += 1
        self.tracks = [tr for tr in self.tracks if tr.missed <= self.max_missed]

        for index in unmatched:
            cluster = clusters[index]
            self.tracks.append(Track(self._next_id, cluster.x, cluster.y,
                                     hits=1, last_t=t, radius=cluster.radius,
                                     history=[(t, cluster.x, cluster.y)]))
            self._next_id += 1
        return list(self.tracks)


def time_to_collision(track: Track, robot_radius: float = 0.25,
                      safety_margin: float = 0.10,
                      max_ttc: float = 10.0,
                      max_dynamic_radius: float = 0.8,
                      min_closing_speed: float = 0.05) -> float:
    """Time until the robot's safety circle is hit, or inf if receding/static.

    Blobs larger than ``max_dynamic_radius`` (walls, vehicle sides) are not
    modelled as circles and are excluded from TTC.
    """
    if track.radius > max_dynamic_radius:
        return float("inf")
    px, py = track.x, track.y
    vx, vy = track.vx, track.vy
    distance = math.hypot(px, py)
    if distance < 1e-6:
        return 0.0
    closing_speed = -(px * vx + py * vy) / distance
    if closing_speed < min_closing_speed:
        return float("inf")
    radius = robot_radius + safety_margin + track.radius
    a = vx * vx + vy * vy
    b = 2.0 * (px * vx + py * vy)
    c = px * px + py * py - radius * radius
    if c <= 0.0:
        return 0.0
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return float("inf")
    t = (-b - math.sqrt(disc)) / (2.0 * a)
    return t if 0.0 <= t <= max_ttc else float("inf")


@dataclass
class WarningGate:
    on_ttc: float = 2.5
    off_ttc: float = 3.5
    active: bool = False

    def update(self, ttc: float) -> bool:
        if self.active:
            if ttc > self.off_ttc:
                self.active = False
        elif ttc < self.on_ttc:
            self.active = True
        return self.active

