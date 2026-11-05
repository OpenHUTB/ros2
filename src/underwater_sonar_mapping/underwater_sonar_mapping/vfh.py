"""Vector Field Histogram (VFH+) steering for a scanning sonar.

The pipeline is the classic three stages:

1. :func:`polar_histogram` turns the sonar returns into a polar obstacle
   density, widening each echo by the vehicle radius so a narrow gap that the
   hull cannot pass stays blocked.
2. :func:`smooth_histogram` applies a circular low-pass kernel, the standard
   VFH smoothing that removes single-beam spikes.
3. :func:`select_direction` picks the free valley that is closest to the goal
   direction, with hysteresis so the vehicle does not oscillate between two
   similar gaps.

Everything here is pure ``numpy`` and ROS-free, so it can be unit tested
without a simulator.
"""
from dataclasses import dataclass
import math

import numpy as np

SMOOTHING_KERNEL = (1.0, 2.0, 4.0, 2.0, 1.0)


def wrap_deg(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def angle_difference_deg(a, b):
    """Signed smallest rotation from ``b`` to ``a`` in degrees."""
    return wrap_deg(a - b)


def sector_angles(sectors):
    """Centre angle of every sector in degrees."""
    width = 360.0 / sectors
    return -180.0 + (np.arange(sectors) + 0.5) * width


def polar_histogram(bearings, ranges, sectors=72, max_range=30.0,
                    robot_radius=0.6, safety_margin=0.5):
    """Return the polar obstacle density in [0, 1], one value per sector.

    ``bearings`` are body-frame angles in radians, ``ranges`` the matching echo
    distances in metres.  A return at ``max_range`` means "nothing heard" and is
    ignored.
    """
    if sectors < 8:
        raise ValueError('sectors must be at least 8')
    bearings = np.asarray(bearings, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    if bearings.shape != ranges.shape:
        raise ValueError('bearings and ranges must have the same shape')

    width = 2.0 * math.pi / sectors
    hist = np.zeros(sectors, dtype=float)
    inflation = robot_radius + safety_margin

    for bearing, distance in zip(bearings, ranges):
        if not np.isfinite(distance) or distance >= max_range or distance <= 0.0:
            continue
        weight = (1.0 - distance / max_range) ** 2
        if distance <= inflation:
            # Already inside the inflated hull: block the sector outright.
            spread = width
            weight = 1.0
        else:
            spread = min(math.asin(min(1.0, inflation / distance)), width * 2)
        lo = int(math.floor((bearing - spread + math.pi) / width))
        hi = int(math.ceil((bearing + spread + math.pi) / width))
        for index in range(lo, hi + 1):
            hist[index % sectors] = max(hist[index % sectors], weight)

    peak = hist.max()
    if peak > 0:
        hist = hist / peak
    return hist


def smooth_histogram(hist, kernel=SMOOTHING_KERNEL):
    """Circular smoothing with the standard VFH kernel, kept in [0, 1]."""
    hist = np.asarray(hist, dtype=float)
    kernel = np.asarray(kernel, dtype=float)
    if hist.size < kernel.size:
        return hist.copy()
    total = kernel.sum()
    smoothed = np.zeros_like(hist)
    offset = kernel.size // 2
    for shift, gain in enumerate(kernel):
        smoothed += gain * np.roll(hist, shift - offset)
    return smoothed / total


@dataclass(frozen=True)
class Steering:
    """Result of one VFH steering decision."""

    angle_deg: float      # body-frame steering direction
    sector: int           # chosen histogram sector
    valley_deg: float     # angular width of the chosen free valley
    blocked: bool         # True when no valley is wide enough


def _free_runs(free):
    """Contiguous runs of free sectors, handling wrap-around."""
    n = free.size
    if free.all():
        return [list(range(n))]
    if not free.any():
        return []
    start = int(np.argmin(free))          # first blocked sector
    rolled = np.roll(free, -start)
    runs, i = [], 0
    while i < n:
        if rolled[i]:
            j = i
            while j < n and rolled[j]:
                j += 1
            runs.append([(k + start) % n for k in range(i, j)])
            i = j
        else:
            i += 1
    return runs


def _run_closest_sector(run, angles, target_deg):
    """Sector inside ``run`` whose direction is closest to ``target_deg``."""
    best, best_error = run[0], math.inf
    for sector in run:
        error = abs(angle_difference_deg(angles[sector], target_deg))
        if error < best_error:
            best, best_error = sector, error
    return best, best_error


def select_direction(hist, goal_bearing_deg, threshold=0.35, min_valley_deg=15.0,
                     hysteresis_deg=15.0, previous_deg=None):
    """Choose a steering direction from the (smoothed) histogram.

    ``goal_bearing_deg`` and ``previous_deg`` are body-frame angles.  Returns
    ``None`` when every valley is narrower than ``min_valley_deg``, which the
    caller should treat as "blocked".
    """
    hist = np.asarray(hist, dtype=float)
    sectors = hist.size
    angles = sector_angles(sectors)
    width_deg = 360.0 / sectors

    free = hist <= threshold
    runs = [run for run in _free_runs(free) if len(run) * width_deg >= min_valley_deg]
    if not runs:
        return Steering(wrap_deg(previous_deg if previous_deg is not None else 0.0),
                        -1, 0.0, True)

    scored = []
    for run in runs:
        sector, error = _run_closest_sector(run, angles, goal_bearing_deg)
        scored.append((error, sector, run))
    scored.sort(key=lambda item: item[0])
    best_error, best_sector, best_run = scored[0]

    if previous_deg is not None:
        previous_sector = int(((previous_deg + 180.0) % 360.0) / width_deg) % sectors
        if free[previous_sector]:
            for error, sector, run in scored:
                if run is best_run:
                    continue
                if previous_sector in run and error - best_error <= hysteresis_deg:
                    best_sector, best_run = sector, run
                    break

    return Steering(wrap_deg(angles[best_sector]), best_sector,
                    len(best_run) * width_deg, False)


def clearance_along(bearings, ranges, angle_deg, window_deg=10.0, max_range=30.0):
    """Nearest echo within ``window_deg`` of a body-frame steering direction.

    Speed must follow the clearance **in the chosen direction**, not the global
    nearest echo: a vehicle squeezed past one block would otherwise stop even
    though the valley it selected is wide open.
    """
    bearings = np.asarray(bearings, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    if bearings.shape != ranges.shape:
        raise ValueError('bearings and ranges must have the same shape')

    difference = np.abs(((np.degrees(bearings) - angle_deg + 180.0) % 360.0) - 180.0)
    local = ranges[difference <= window_deg]
    hits = local[local < max_range]
    if hits.size:
        return float(hits.min())
    hits = ranges[ranges < max_range]
    return float(hits.min()) if hits.size else float(max_range)


def speed_scale(clearance, slow_distance=2.5, stop_distance=0.8):
    """Scale the commanded speed by the nearest echo distance."""
    if clearance <= stop_distance:
        return 0.0
    if clearance >= slow_distance:
        return 1.0
    return (clearance - stop_distance) / (slow_distance - stop_distance)
