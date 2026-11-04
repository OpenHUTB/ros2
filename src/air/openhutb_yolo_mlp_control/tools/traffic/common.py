"""Shared telemetry and coordinate helpers; no simulator imports."""

import argparse
import json
import math
import os
import statistics
import time
from pathlib import Path

REFERENCE_LIMIT = 20


def write_json(path, data):
    """Replace a complete snapshot, retrying Windows sharing violations."""
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
    for attempt in range(20):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(0.01)


def read_live(directory, max_age=2.0):
    for attempt in range(20):
        try:
            data = json.loads((Path(directory) / 'live.json').read_text(encoding='utf-8'))
            break
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(0.01)
    age = time.time() - data['time']
    if not -1.0 <= age <= max_age:
        raise RuntimeError('Vehicle telemetry is stale or clocks disagree')
    return data


def calibration_offset(pairs, tolerance=0.25):
    """Fit translation after CARLA z-up to AirSim z-down conversion."""
    if len(pairs) < 5:
        raise ValueError('At least five finite shared scene references are required')
    offsets = [[b[0] - a[0], b[1] - a[1], b[2] + a[2]] for a, b in pairs]
    if not all(math.isfinite(x) for row in offsets for x in row):
        raise ValueError('Nonfinite calibration reference')
    offset = [statistics.median(row[i] for row in offsets) for i in range(3)]
    residual = max(math.dist(row, offset) for row in offsets)
    if residual > tolerance:
        raise ValueError(f'Coordinate calibration residual {residual:.3f} m exceeds tolerance')
    return offset, residual


def following_point(vehicle, offset, distance, height):
    yaw = math.radians(vehicle['yaw_deg'])
    return [vehicle['x'] + offset[0] - distance * math.cos(yaw),
            vehicle['y'] + offset[1] - distance * math.sin(yaw),
            -vehicle['z'] + offset[2] - height]


def step_towards(current, target, maximum):
    distance = math.dist(current, target)
    scale = min(1.0, maximum / max(distance, 1e-9))
    return [a + (b - a) * scale for a, b in zip(current, target)]


def velocity_towards(current, target, maximum_speed, gain=1.0,
                     previous=None, dt=0.2, max_acceleration=3.0,
                     target_velocity=None):
    """Return a bounded, acceleration-limited world-frame velocity command."""
    delta = [b - a for a, b in zip(current, target)]
    feedforward = (target_velocity if target_velocity is not None
                   else [0.0] * len(delta))
    desired = [feedforward[i] + gain * delta[i] for i in range(len(delta))]
    desired_length = math.sqrt(sum(value * value for value in desired))
    if desired_length > maximum_speed:
        desired = [value * maximum_speed / desired_length for value in desired]
    if previous is None:
        previous = [0.0] * len(desired)
    change = [new - old for old, new in zip(previous, desired)]
    change_length = math.sqrt(sum(value * value for value in change))
    maximum_change = max_acceleration * min(max(dt, 0.0), 0.25)
    if change_length > maximum_change:
        scale = maximum_change / change_length
        change = [value * scale for value in change]
    return [old + value for old, value in zip(previous, change)]


def follow_phase(current, target, altitude_tolerance=2.0):
    return 'climb' if abs(current[2] - target[2]) > altitude_tolerance else 'follow'


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Must be positive and finite')
    return number


def limited_references(references, limit=REFERENCE_LIMIT):
    return list(references[:limit])


def actual_sampling_rate(sample_count, elapsed):
    if sample_count < 2 or elapsed <= 0:
        return 0.0
    return (sample_count - 1) / elapsed


def follow_step(current, target, maximum):
    """Reserve climb progress while retaining a bounded horizontal approach."""
    if abs(current[2] - target[2]) <= 2:
        return step_towards(current, target, maximum)
    dz = max(-maximum / math.sqrt(2), min(maximum / math.sqrt(2), target[2] - current[2]))
    horizontal = step_towards(current[:2], target[:2], math.sqrt(max(0, maximum ** 2 - dz ** 2)))
    return [*horizontal, current[2] + dz]


def pose_clearance(current, vehicle, offset, minimum=2.0):
    """Check altitude above the target road, not the unready flight controller.

    This is a coarse start guard, not local terrain or obstacle sensing.
    """
    road_z = -vehicle['z'] + offset[2]
    clearance = road_z - current[2]
    if not math.isfinite(clearance) or clearance < minimum:
        raise RuntimeError('Pose follow requires at least 2 m above the target road; '
                           'move the drone to a safe height first')
    return clearance
