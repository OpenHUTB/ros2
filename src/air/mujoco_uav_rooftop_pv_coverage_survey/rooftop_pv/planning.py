"""Local route waypoint geometry / 局部航线目标点几何。"""

from __future__ import annotations

import math


LOOKAHEAD_METRES = 2.0


def planning_features(cross_track: float, heading: float, distance: float) -> tuple[float, float, float]:
    """Observed route-relative state; fixed scaling uses no fitted test statistics."""
    if not all(math.isfinite(value) for value in (cross_track, heading, distance)):
        raise ValueError("planning state must be finite")
    return cross_track, math.atan2(math.sin(heading), math.cos(heading)), max(0.0, min(distance, LOOKAHEAD_METRES))


def route_waypoint_target(cross_track: float, heading: float, distance: float) -> tuple[float, float]:
    """Expert lookahead waypoint in body metres, distinct from velocity commands.

    The target lies on the current route segment, at most two metres ahead.
    航线专家在当前航段上选择前方两米内的目标，输出机体系前向、左向位移。
    """
    cross_track, heading, ahead = planning_features(cross_track, heading, distance)
    return (ahead * math.cos(heading) - cross_track * math.sin(heading),
            -ahead * math.sin(heading) - cross_track * math.cos(heading))
