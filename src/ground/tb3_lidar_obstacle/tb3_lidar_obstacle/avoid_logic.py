# -*- coding: utf-8 -*-
"""Pure-Python control logic for the TurtleBot3 lidar obstacle demo.

This module intentionally has no rclpy / ROS imports so the decision logic
can be tested with the standard library alone (no Gazebo, no ROS workspace).
``obstacle_avoid.py`` only adapts LaserScan messages to these functions.
"""

import math

SAFE_DIST = 0.5
FRONT_HALF_ANGLE_DEG = 15.0
SIDE_HALF_ANGLE_DEG = 30.0
LINEAR_SPEED = 0.2
TURN_SPEED = 0.3
SCAN_TIMEOUT_S = 0.5
STOP = (0.0, 0.0, "stop")


def _finite_ranges(ranges, range_min=0.0, range_max=float("inf")):
    """Yield (index, distance) for physically valid LaserScan samples."""
    lo = max(0.0, float(range_min))
    hi = float(range_max) if math.isfinite(range_max) else float("inf")
    for i, r in enumerate(ranges):
        try:
            v = float(r)
        except (TypeError, ValueError):
            continue
        if math.isfinite(v) and lo < v < hi:
            yield i, v


def sector_min(ranges, angle_min, angle_increment, center_deg,
               half_width_deg, range_min=0.0, range_max=float("inf")):
    """Minimum valid range inside an angular sector (degrees, 0 = front).

    Indices are derived from ``angle_min + i * angle_increment`` instead of
    assuming ``ranges[0]`` is straight ahead; an empty / invalid sector is
    reported as infinity.
    """
    inc = float(angle_increment)
    if inc <= 0.0 or not ranges:
        return float("inf")
    lo = math.radians(center_deg - half_width_deg)
    hi = math.radians(center_deg + half_width_deg)
    i0 = int(round((lo - float(angle_min)) / inc))
    i1 = int(round((hi - float(angle_min)) / inc))
    best = float("inf")
    for i, v in _finite_ranges(ranges, range_min, range_max):
        if i0 <= i <= i1:
            best = min(best, v)
    return best


def decide(front, left=None, right=None, safe_dist=SAFE_DIST,
           turn_speed=TURN_SPEED, linear_speed=LINEAR_SPEED,
           side_choice=True):
    """Return ``(linear_x, angular_z, state)`` for one decision step.

    When both sides are supplied the clearer side wins (left/right average
    range is not used here: ``sector_min`` already gives the nearest obstacle
    in that sector).  Keeping this a plain tuple makes the watchdog, the ROS
    node and the unit tests share exactly one truth table.
    """
    f = front if math.isfinite(front) else float("inf")
    if f >= safe_dist:
        return (linear_speed, 0.0, "forward")
    if side_choice and left is not None and right is not None:
        l = left if math.isfinite(left) else float("inf")
        r = right if math.isfinite(right) else float("inf")
        if l > r + 1e-6:
            return (0.0, turn_speed, "turn_left")
        if r > l + 1e-6:
            return (0.0, -turn_speed, "turn_right")
    return (0.0, turn_speed, "turn_left")


def scan_to_decision(ranges, angle_min, angle_increment,
                     range_min=0.0, range_max=float("inf"),
                     safe_dist=SAFE_DIST):
    """Turn one LaserScan's ranges into a control tuple."""
    front = sector_min(ranges, angle_min, angle_increment, 0.0,
                       FRONT_HALF_ANGLE_DEG, range_min, range_max)
    left = sector_min(ranges, angle_min, angle_increment,
                      SIDE_HALF_ANGLE_DEG, FRONT_HALF_ANGLE_DEG,
                      range_min, range_max)
    right = sector_min(ranges, angle_min, angle_increment,
                       -SIDE_HALF_ANGLE_DEG, FRONT_HALF_ANGLE_DEG,
                       range_min, range_max)
    return decide(front, left, right, safe_dist=safe_dist)
