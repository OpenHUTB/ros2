"""Deterministic 2-D simulation for the dynamic-obstacle TTC module."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    from .ttc_core import Cluster, Tracker, WarningGate, cluster_scan, time_to_collision
except ImportError:  # direct execution
    from ttc_core import Cluster, Tracker, WarningGate, cluster_scan, time_to_collision


@dataclass
class MovingCircle:
    x: float
    y: float
    vx: float
    vy: float
    r: float = 0.25


@dataclass
class Scenario:
    name: str
    circles: List[MovingCircle]
    duration: float = 8.0
    description: str = ""


def raycast(circles: Sequence[MovingCircle], n_beams: int = 180,
            max_range: float = 6.0) -> np.ndarray:
    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    dirs = np.column_stack([np.cos(angles), np.sin(angles)])
    ranges = np.full(n_beams, np.inf, dtype=float)
    for circle in circles:
        ocx, ocy = circle.x, circle.y
        proj = dirs[:, 0] * ocx + dirs[:, 1] * ocy
        perp2 = ocx * ocx + ocy * ocy - proj * proj
        disc = circle.r * circle.r - perp2
        hit = (disc >= 0.0) & (proj > 0.0)
        t = np.where(hit, proj - np.sqrt(np.maximum(disc, 0.0)), np.inf)
        ranges = np.minimum(ranges, t)
    return ranges


def crossing() -> Scenario:
    return Scenario("crossing", [MovingCircle(4.0, -3.0, -0.6, 0.45)],
                    description="斜穿机器人前方，约 6.7 s 到原点")


def head_on() -> Scenario:
    return Scenario("head_on", [MovingCircle(5.0, 0.0, -0.8, 0.0)],
                    description="正面接近，约 6.2 s 到原点")


def parallel() -> Scenario:
    return Scenario("parallel", [MovingCircle(1.5, -4.0, 0.0, 0.7)],
                    description="平行通过，横向 1.5 m，无碰撞风险")


def two_obstacles() -> Scenario:
    return Scenario("two", [MovingCircle(5.0, 0.0, -0.8, 0.0),
                            MovingCircle(-1.5, -4.0, 0.0, 0.6)],
                    description="正面接近 + 左侧平行通过")


def receding() -> Scenario:
    return Scenario("receding", [MovingCircle(2.0, 0.0, 0.5, 0.0)],
                    description="远离机器人，不应告警")


ALL_SCENARIOS = {"crossing": crossing, "head_on": head_on, "parallel": parallel,
                 "two": two_obstacles, "receding": receding}


def run_scenario(scenario: Scenario, dt: float = 0.1) -> Dict[str, object]:
    tracker = Tracker()
    gate = WarningGate()
    warning_at = None
    min_ttc = float("inf")
    min_distance = float("inf")
    first_track = None
    max_tracks = 0
    collision_time = None
    frames = int(scenario.duration / dt)

    for step in range(frames):
        t = step * dt
        circles = [MovingCircle(c.x + c.vx * t, c.y + c.vy * t,
                                c.vx, c.vy, c.r) for c in scenario.circles]
        impact = min((math.hypot(c.x, c.y) - (0.25 + 0.10 + c.r) for c in circles),
                     default=float("inf"))
        if impact <= 0.0:
            min_distance = 0.0
            collision_time = t
            break
        min_distance = min(min_distance, impact + 0.35)
        ranges = raycast(circles)
        clusters: List[Cluster] = cluster_scan(ranges, -math.pi,
                                               2.0 * math.pi / len(ranges))
        tracks = tracker.update(clusters, t)
        max_tracks = max(max_tracks, len(tracks))
        if tracks and first_track is None:
            first_track = t

        best = min((time_to_collision(tr) for tr in tracks), default=float("inf"))
        min_ttc = min(min_ttc, best)
        active = gate.update(best)
        if active and warning_at is None:
            warning_at = t

    has_threat = scenario.name in ("crossing", "head_on", "two")
    lead = None
    if warning_at is not None and collision_time is not None:
        lead = round(collision_time - warning_at, 2)
    return {
        "scenario": scenario.name,
        "description": scenario.description,
        "warning_at_s": warning_at,
        "collision_at_s": collision_time,
        "warning_lead_s": lead,
        "min_ttc_s": round(min_ttc, 3) if math.isfinite(min_ttc) else None,
        "min_distance_m": round(min_distance, 3),
        "first_track_s": first_track,
        "max_tracks": max_tracks,
        "expected_warning": has_threat,
        "false_positive": bool(not has_threat and warning_at is not None),
        "warned": warning_at is not None,
    }


def run_benchmark() -> List[Dict[str, object]]:
    return [run_scenario(factory()) for factory in ALL_SCENARIOS.values()]


def run_timeline(scenario: Scenario, dt: float = 0.1) -> List[Dict[str, object]]:
    """Return per-frame (t, ttc, warning) for plotting."""
    tracker = Tracker()
    gate = WarningGate()
    timeline = []
    for step in range(int(scenario.duration / dt)):
        t = step * dt
        circles = [MovingCircle(c.x + c.vx * t, c.y + c.vy * t,
                                c.vx, c.vy, c.r) for c in scenario.circles]
        if min((math.hypot(c.x, c.y) - (0.25 + 0.10 + c.r) for c in circles),
               default=float("inf")) <= 0.0:
            break
        ranges = raycast(circles)
        clusters = cluster_scan(ranges, -math.pi, 2.0 * math.pi / len(ranges))
        tracks = tracker.update(clusters, t)
        best = min((time_to_collision(tr) for tr in tracks), default=float("inf"))
        active = gate.update(best)
        timeline.append({"t": t, "ttc": best if math.isfinite(best) else None,
                         "warning": active})
    return timeline


def main() -> None:
    rows = run_benchmark()
    print(f"{'scenario':10s} {'warn@':>6} {'lead':>6} {'tracks':>6} {'false+':>6}")
    for row in rows:
        print(f"{row['scenario']:10s} {str(row['warning_at_s']):>6} "
              f"{str(row['warning_lead_s']):>6} {row['max_tracks']:>6} "
              f"{str(row['false_positive']):>6}")


if __name__ == "__main__":
    main()



