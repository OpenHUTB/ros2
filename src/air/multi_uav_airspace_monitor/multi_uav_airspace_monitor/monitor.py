"""Geofence checks and pairwise closest-point-of-approach monitoring."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from .geometry import distance_to_polygon, point_in_polygon, polygon_area


@dataclass
class UAVTrack:
    """Time-parameterised UAV trajectory in a local ENU frame."""

    uav_id: str
    times: np.ndarray
    positions: np.ndarray

    def __post_init__(self) -> None:
        self.times = np.asarray(self.times, dtype=float)
        self.positions = np.asarray(self.positions, dtype=float)
        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("positions must have shape (N, 3)")
        if self.times.ndim != 1 or len(self.times) != len(self.positions):
            raise ValueError("times must be 1-D and match positions")
        if len(self.times) < 2:
            raise ValueError("a track needs at least two samples")
        if np.any(np.diff(self.times) <= 0):
            raise ValueError("times must be strictly increasing")

    @property
    def start(self) -> np.ndarray:
        return self.positions[0]

    @property
    def end(self) -> np.ndarray:
        return self.positions[-1]

    def position_at(self, time_s: float) -> np.ndarray:
        """Linearly interpolate the trajectory at a given time."""
        return np.array(
            [np.interp(time_s, self.times, self.positions[:, axis]) for axis in range(3)]
        )


@dataclass
class Zone:
    """A prism-shaped no-fly zone with an optional horizontal buffer."""

    name: str
    polygon: np.ndarray
    floor_z: float = 0.0
    ceiling_z: float = 120.0
    buffer_m: float = 0.0

    def __post_init__(self) -> None:
        self.polygon = np.asarray(self.polygon, dtype=float)
        if self.polygon.ndim != 2 or self.polygon.shape[1] != 2:
            raise ValueError("zone polygon must have shape (N, 2)")
        if len(self.polygon) < 3:
            raise ValueError("zone polygon needs at least three vertices")
        if self.ceiling_z <= self.floor_z:
            raise ValueError("ceiling_z must be greater than floor_z")

    @property
    def area_m2(self) -> float:
        return polygon_area(self.polygon)

    def contains(self, position: Sequence[float], include_buffer: bool = False) -> bool:
        x, y, z = float(position[0]), float(position[1]), float(position[2])
        if not (self.floor_z <= z <= self.ceiling_z):
            return False
        if point_in_polygon((x, y), self.polygon):
            return True
        return include_buffer and distance_to_polygon((x, y), self.polygon) <= self.buffer_m


@dataclass
class Alert:
    kind: str
    severity: str
    uav_id: str
    time_s: float
    message: str
    distance_m: float | None = None
    zone: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "severity": self.severity,
            "uav_id": self.uav_id,
            "time_s": round(float(self.time_s), 3),
            "message": self.message,
            "distance_m": None if self.distance_m is None else round(float(self.distance_m), 3),
            "zone": self.zone,
        }


@dataclass
class Conflict:
    uav_a: str
    uav_b: str
    time_s: float
    separation_m: float
    severity: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    advisory: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "uav_a": self.uav_a,
            "uav_b": self.uav_b,
            "time_s": round(float(self.time_s), 3),
            "separation_m": round(float(self.separation_m), 3),
            "severity": self.severity,
            "position": [round(float(value), 3) for value in self.position],
            "advisory": self.advisory,
        }


def geofence_alerts(track: UAVTrack, zones: Iterable[Zone]) -> list[Alert]:
    """Return the first critical/buffer violation for every zone and track."""
    alerts: list[Alert] = []
    for zone in zones:
        first_critical: tuple[float, float] | None = None
        first_buffer: tuple[float, float] | None = None
        for time_s, position in zip(track.times, track.positions):
            if not (zone.floor_z <= float(position[2]) <= zone.ceiling_z):
                continue
            if zone.contains(position):
                first_critical = (float(time_s), 0.0)
                break
            distance = distance_to_polygon(position[:2], zone.polygon)
            if distance <= zone.buffer_m and first_buffer is None:
                first_buffer = (float(time_s), float(distance))

        if first_critical is not None:
            alerts.append(
                Alert(
                    kind="geofence",
                    severity="critical",
                    uav_id=track.uav_id,
                    time_s=first_critical[0],
                    message=f"{track.uav_id} entered no-fly zone {zone.name}",
                    distance_m=0.0,
                    zone=zone.name,
                )
            )
        elif first_buffer is not None:
            alerts.append(
                Alert(
                    kind="geofence",
                    severity="warning",
                    uav_id=track.uav_id,
                    time_s=first_buffer[0],
                    message=f"{track.uav_id} approached no-fly zone {zone.name}",
                    distance_m=first_buffer[1],
                    zone=zone.name,
                )
            )
    return alerts


def validate_window(horizon_s: float | None, step_s: float) -> tuple[float | None, float]:
    """Validate a sampling window; shared by every entry point that samples time.

    ``horizon_s=None`` means "the whole common interval" and is therefore allowed
    here, but a NaN/inf step or a non-positive horizon is a programming error and
    raises ``ValueError`` instead of producing a stretched or empty series.
    """
    if not np.isfinite(step_s) or float(step_s) <= 0.0:
        raise ValueError("step_s must be a positive finite number")
    if horizon_s is None:
        return None, float(step_s)
    if not np.isfinite(horizon_s) or float(horizon_s) <= 0.0:
        raise ValueError("horizon_s must be a positive finite number")
    return float(horizon_s), float(step_s)


def _interpolate_pair(
    track_a: UAVTrack,
    track_b: UAVTrack,
    step_s: float,
    horizon_s: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample the common interval of two tracks, optionally limited to a horizon.

    Samples are taken from the shared start with the requested ``step_s`` (not
    stretched to fit the window); when the window is not a multiple of the step,
    the exact end point is appended so the last interval is simply shorter.
    """
    start = max(float(track_a.times[0]), float(track_b.times[0]))
    end = min(float(track_a.times[-1]), float(track_b.times[-1]))
    if horizon_s is not None:
        end = min(end, start + float(horizon_s))
    if end <= start:
        return np.empty(0), np.empty((0, 3)), np.empty((0, 3))
    span = end - start
    step = float(step_s)
    count = int(np.floor(span / step)) + 1
    times = start + np.arange(count) * step
    if times.size == 0 or times[-1] < end - 1e-9:
        times = np.append(times, end)
    positions_a = np.vstack([track_a.position_at(t) for t in times])
    positions_b = np.vstack([track_b.position_at(t) for t in times])
    return times, positions_a, positions_b


def pairwise_conflicts(
    tracks: Sequence[UAVTrack],
    horizon_s: float = 10.0,
    warning_m: float = 30.0,
    critical_m: float = 10.0,
    step_s: float = 0.25,
) -> list[Conflict]:
    """Detect conflicts inside the first ``horizon_s`` seconds of common flight.

    Samples are truncated to ``[start, start + horizon_s]`` before the closest
    approach is taken. A pair that is already inside the warning distance inside
    the horizon is therefore still reported when the global minimum of the whole
    scenario happens later than the horizon.
    """
    horizon_s, step_s = validate_window(horizon_s, step_s)
    conflicts: list[Conflict] = []
    for index, track_a in enumerate(tracks):
        for track_b in tracks[index + 1:]:
            times, positions_a, positions_b = _interpolate_pair(
                track_a, track_b, step_s, horizon_s
            )
            if len(times) == 0:
                continue
            distances = np.linalg.norm(positions_a - positions_b, axis=1)
            min_index = int(np.argmin(distances))
            min_distance = float(distances[min_index])
            if min_distance > warning_m:
                continue

            severity = "critical" if min_distance <= critical_m else "warning"
            midpoint = 0.5 * (positions_a[min_index] + positions_b[min_index])
            advisory = _resolution_advisory(track_a.uav_id, track_b.uav_id, severity)
            conflicts.append(
                Conflict(
                    uav_a=track_a.uav_id,
                    uav_b=track_b.uav_id,
                    time_s=float(times[min_index]),
                    separation_m=min_distance,
                    severity=severity,
                    position=midpoint,
                    advisory=advisory,
                )
            )
    return sorted(conflicts, key=lambda item: (item.severity != "critical", item.time_s))


def _resolution_advisory(uav_a: str, uav_b: str, severity: str) -> str:
    if severity == "critical":
        return (
            f"{uav_b}: climb 15 m and reduce speed by 20%; "
            f"{uav_a}: keep heading and monitor separation"
        )
    return f"{uav_b}: reduce speed by 10% and maintain visual separation from {uav_a}"


def _separations(
    tracks: Sequence[UAVTrack], step_s: float, horizon_s: float | None
) -> list[float]:
    """Sample every pair on the same time window and collect pairwise distances."""
    distances: list[float] = []
    for index, track_a in enumerate(tracks):
        for track_b in tracks[index + 1:]:
            times, positions_a, positions_b = _interpolate_pair(
                track_a, track_b, step_s, horizon_s
            )
            if len(times):
                distances.extend(
                    np.linalg.norm(positions_a - positions_b, axis=1).tolist()
                )
    return distances


def separation_series(
    tracks: Sequence[UAVTrack],
    step_s: float = 0.25,
    horizon_s: float | None = None,
) -> list[dict[str, Any]]:
    """Per-sample pairwise separations, for CSV export and plotting.

    One row per sampled time step and UAV pair, sorted by ``time_s`` and then by
    the pair order in ``tracks``. Sampling uses the same true-step scheme as the
    conflict detector (``horizon_s``/``step_s`` are validated here too), so a row's
    ``separation_m`` can be compared directly with the warning and critical
    distances.
    """
    horizon_s, step_s = validate_window(horizon_s, step_s)
    rows: list[dict[str, Any]] = []
    for index, track_a in enumerate(tracks):
        for track_b in tracks[index + 1:]:
            times, positions_a, positions_b = _interpolate_pair(
                track_a, track_b, step_s, horizon_s
            )
            if not len(times):
                continue
            distances = np.linalg.norm(positions_a - positions_b, axis=1)
            for time_s, distance_m in zip(times.tolist(), distances.tolist()):
                rows.append(
                    {
                        "time_s": round(float(time_s), 3),
                        "uav_a": track_a.uav_id,
                        "uav_b": track_b.uav_id,
                        "separation_m": round(float(distance_m), 3),
                    }
                )
    rows.sort(key=lambda row: (row["time_s"], row["uav_a"], row["uav_b"]))
    return rows


def parse_scenario(
    scenario: dict[str, Any],
) -> tuple[list[UAVTrack], list[Zone], float, float]:
    """Turn a scenario dictionary into ``(tracks, zones, horizon_s, step_s)``."""
    if not isinstance(scenario, dict):
        raise ValueError("scenario must be a mapping")
    uavs = scenario.get("uavs")
    if not uavs:
        raise ValueError("scenario must contain at least one UAV track in 'uavs'")

    zones = [
        Zone(
            name=item["name"],
            polygon=item["polygon"],
            floor_z=item.get("floor_z", 0.0),
            ceiling_z=item.get("ceiling_z", 120.0),
            buffer_m=item.get("buffer_m", 0.0),
        )
        for item in scenario.get("zones", [])
    ]
    tracks = [
        UAVTrack(
            uav_id=item["id"],
            times=item["times"],
            positions=item["positions"],
        )
        for item in uavs
    ]
    horizon_s = float(scenario.get("horizon_s", 10.0))
    step_s = float(scenario.get("step_s", 0.25))
    horizon_s, step_s = validate_window(horizon_s, step_s)
    return tracks, zones, horizon_s, step_s


def separation_rows_for_scenario(scenario: dict[str, Any]) -> list[dict[str, Any]]:
    """Convenience wrapper: scenario dictionary in, CSV-ready rows out."""
    tracks, _zones, horizon_s, step_s = parse_scenario(scenario)
    return separation_series(tracks, step_s=step_s, horizon_s=horizon_s)


def evaluate_scenario(scenario: dict[str, Any]) -> dict[str, Any]:
    """Evaluate a complete scenario dictionary and return a JSON-ready summary.

    ``minimum_separation_m`` is measured inside the same ``horizon_s`` window the
    conflict detector uses, so the number never contradicts ``conflicts``. The
    whole common interval is reported separately as ``minimum_separation_full_m``.
    """
    tracks, zones, horizon_s, step_s = parse_scenario(scenario)

    alerts: list[Alert] = []
    for track in tracks:
        alerts.extend(geofence_alerts(track, zones))

    conflicts = pairwise_conflicts(
        tracks,
        horizon_s=horizon_s,
        warning_m=float(scenario.get("warning_distance_m", 30.0)),
        critical_m=float(scenario.get("critical_distance_m", 10.0)),
        step_s=step_s,
    )

    window_separations = _separations(tracks, step_s, horizon_s)
    full_separations = _separations(tracks, step_s, None)

    critical = any(alert.severity == "critical" for alert in alerts) or any(
        conflict.severity == "critical" for conflict in conflicts
    )
    warning = any(alert.severity == "warning" for alert in alerts) or any(
        conflict.severity == "warning" for conflict in conflicts
    )
    if critical:
        status = "critical"
    elif warning:
        status = "warning"
    else:
        status = "safe"

    return {
        "status": status,
        "uav_count": len(tracks),
        "zone_count": len(zones),
        "horizon_s": round(horizon_s, 3),
        "minimum_separation_m": round(min(window_separations), 3)
        if window_separations
        else None,
        "minimum_separation_full_m": round(min(full_separations), 3)
        if full_separations
        else None,
        "alerts": [alert.as_dict() for alert in alerts],
        "conflicts": [conflict.as_dict() for conflict in conflicts],
    }
