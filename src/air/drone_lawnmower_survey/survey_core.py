"""Pure-Python geometry helpers for a lawnmower survey mission."""

import math


def _positive_finite(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("%s must be a positive finite number" % name)
    return value


def build_lawnmower_path(length, width, lane_spacing, altitude):
    """Return local NED waypoints that cover a centered rectangle.

    ``length`` is the x-axis extent and ``width`` is the y-axis extent.
    The returned z coordinate is negative because AirSim uses NED axes.
    The actual lane spacing never exceeds ``lane_spacing`` and both
    rectangle boundaries are included.
    """

    length = _positive_finite("length", length)
    width = _positive_finite("width", width)
    lane_spacing = _positive_finite("lane_spacing", lane_spacing)
    altitude = _positive_finite("altitude", altitude)

    lane_intervals = max(1, int(math.ceil(width / lane_spacing)))
    actual_spacing = width / lane_intervals
    x_min = -length / 2.0
    x_max = length / 2.0
    y_min = -width / 2.0
    flight_z = -altitude

    path = []
    for lane_index in range(lane_intervals + 1):
        y = y_min + lane_index * actual_spacing
        if lane_index % 2 == 0:
            path.extend(((x_min, y, flight_z), (x_max, y, flight_z)))
        else:
            path.extend(((x_max, y, flight_z), (x_min, y, flight_z)))
    return path


def translate_path(path, offset_x=0.0, offset_y=0.0, offset_z=0.0):
    """Translate local waypoints into AirSim's world NED frame."""

    offsets = (float(offset_x), float(offset_y), float(offset_z))
    if not all(math.isfinite(value) for value in offsets):
        raise ValueError("path offsets must be finite numbers")
    return [
        (x + offsets[0], y + offsets[1], z + offsets[2])
        for x, y, z in path
    ]


def path_length(path, start=None):
    """Calculate the 3D length of a waypoint sequence."""

    points = list(path)
    if start is not None:
        points.insert(0, start)
    return sum(
        math.sqrt(
            (current[0] - previous[0]) ** 2
            + (current[1] - previous[1]) ** 2
            + (current[2] - previous[2]) ** 2
        )
        for previous, current in zip(points, points[1:])
    )
