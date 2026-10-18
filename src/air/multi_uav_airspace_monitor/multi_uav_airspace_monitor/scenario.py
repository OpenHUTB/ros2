"""Synthetic scenarios used by the offline demo and regression tests."""

from __future__ import annotations

import numpy as np


def build_demo_scenario(duration_s: float = 40.0, dt_s: float = 0.5) -> dict:
    """Four UAVs: one head-on conflict and one geofence intrusion."""
    times = np.arange(0.0, duration_s + dt_s, dt_s)
    progress = times / duration_s

    def line(start: tuple[float, float, float], end: tuple[float, float, float]):
        start_array = np.asarray(start, dtype=float)
        end_array = np.asarray(end, dtype=float)
        positions = start_array + progress[:, None] * (end_array - start_array)
        return positions.tolist()

    uavs = [
        {
            "id": "uav_alpha",
            "times": times.tolist(),
            "positions": line((-150.0, -60.0, 60.0), (150.0, 60.0, 60.0)),
        },
        {
            "id": "uav_beta",
            "times": times.tolist(),
            "positions": line((150.0, -60.0, 60.0), (-150.0, 60.0, 60.0)),
        },
        {
            "id": "uav_gamma",
            "times": times.tolist(),
            "positions": line((-40.0, 130.0, 80.0), (55.0, 20.0, 80.0)),
        },
        {
            "id": "uav_delta",
            "times": times.tolist(),
            "positions": line((100.0, -130.0, 100.0), (145.0, -90.0, 100.0)),
        },
    ]

    return {
        "name": "four_uav_airspace_demo",
        "horizon_s": 20.0,
        "warning_distance_m": 30.0,
        "critical_distance_m": 10.0,
        "step_s": 0.25,
        "zones": [
            {
                "name": "city_center_restricted",
                "polygon": [[20.0, -30.0], [90.0, -30.0], [90.0, 60.0], [20.0, 60.0]],
                "floor_z": 0.0,
                "ceiling_z": 120.0,
                "buffer_m": 20.0,
            }
        ],
        "uavs": uavs,
    }
