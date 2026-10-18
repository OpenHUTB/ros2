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


def _interpolate_pair(
    track_a: UAVTrack,
    track_b: UAVTrack,
    step_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = max(float(track_a.times[0]), float(track_b.times[0]))
    end = min(float(track_a.times[-1]), float(track_b.times[-1]))
    if end <= start:
        return np.empty(0), np.empty((0, 3)), np.empty((0, 3))
    count = max(2, int(np.floor((end - start) / step_s)) + 1)
    times = np.linspace(start, end, count)
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
    """Detect conflicts using the minimum separation over the common horizon."""
    conflicts: list[Conflict] = []
    for index, track_a in enumerate(tracks):
        for track_b in tracks[index + 1 :]:
            times, positions_a, positions_b = _interpolate_pair(track_a, track_b, step_s)
            if len(times) == 0:
                continue
            distances = np.linalg.norm(positions_a - positions_b, axis=1)
            min_index = int(np.argmin(distances))
            min_distance = float(distances[min_index])
            if min_distance > warning_m:
                continue
            if times[min_index] - times[0] > horizon_s + 1e-9:
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


def evaluate_scenario(scenario: dict[str, Any]) -> dict[str, Any]:
    """Evaluate a complete scenario dictionary and return a JSON-ready summary."""
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
        for item in scenario["uavs"]
    ]

    alerts: list[Alert] = []
    for track in tracks:
        alerts.extend(geofence_alerts(track, zones))

    conflicts = pairwise_conflicts(
        tracks,
        horizon_s=float(scenario.get("horizon_s", 10.0)),
        warning_m=float(scenario.get("warning_distance_m", 30.0)),
        critical_m=float(scenario.get("critical_distance_m", 10.0)),
        step_s=float(scenario.get("step_s", 0.25)),
    )

    all_separations = []
    for index, track_a in enumerate(tracks):
        for track_b in tracks[index + 1 :]:
            times, positions_a, positions_b = _interpolate_pair(track_a, track_b, 0.5)
            if len(times):
                all_separations.extend(
                    np.linalg.norm(positions_a - positions_b, axis=1).tolist()
                )

    status = "safe"
    if conflicts or any(alert.severity == "critical" for alert in alerts):
        status = "critical"
    elif any(alert.severity == "warning" for alert in alerts):
        status = "warning"

    return {
        "status": status,
        "uav_count": len(tracks),
        "zone_count": len(zones),
        "minimum_separation_m": round(min(all_separations), 3) if all_separations else None,
        "alerts": [alert.as_dict() for alert in alerts],
        "conflicts": [conflict.as_dict() for conflict in conflicts],
    }
