"""Boustrophedon survey planner with GSD and overlap constraints."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .camera import CameraSpec
from .geometry import (
    line_intervals,
    point_in_polygon,
    polyline_length,
    principal_angle,
    rotate_points,
    sample_coverage,
)


@dataclass
class SurveyPlan:
    altitude_m: float
    gsd_m: float
    footprint_width_m: float
    footprint_height_m: float
    line_spacing_m: float
    shot_spacing_m: float
    line_count: int
    image_count: int
    flight_distance_m: float
    flight_time_s: float
    coverage_ratio: float
    waypoints_enu: np.ndarray
    scan_segments: list[tuple[tuple[float, float], tuple[float, float]]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    origin_lat: float = 28.22
    origin_lon: float = 112.94

    def as_dict(self) -> dict:
        return {
            "altitude_m": round(self.altitude_m, 4),
            "gsd_m": round(self.gsd_m, 6),
            "footprint_width_m": round(self.footprint_width_m, 4),
            "footprint_height_m": round(self.footprint_height_m, 4),
            "line_spacing_m": round(self.line_spacing_m, 4),
            "shot_spacing_m": round(self.shot_spacing_m, 4),
            "line_count": self.line_count,
            "image_count": self.image_count,
            "flight_distance_m": round(self.flight_distance_m, 3),
            "flight_time_s": round(self.flight_time_s, 3),
            "coverage_ratio": round(self.coverage_ratio, 4),
            "waypoint_count": int(len(self.waypoints_enu)),
            "warnings": list(self.warnings),
        }

    def waypoints_latlon(self) -> list[tuple[float, float, float]]:
        result = []
        cos_lat = math.cos(math.radians(self.origin_lat))
        for east, north in self.waypoints_enu:
            lat = self.origin_lat + math.degrees(north / 6_371_000.0)
            lon = self.origin_lon + math.degrees(east / (6_371_000.0 * cos_lat))
            result.append((lat, lon, self.altitude_m))
        return result


class SurveyPlanner:
    def __init__(
        self,
        camera: CameraSpec,
        target_gsd_m: float,
        forward_overlap: float = 0.8,
        side_overlap: float = 0.7,
        speed_mps: float = 8.0,
        turn_time_s: float = 2.0,
        max_flight_time_s: float = 1500.0,
        origin_lat: float = 28.22,
        origin_lon: float = 112.94,
    ) -> None:
        if not 0.0 <= forward_overlap < 1.0:
            raise ValueError("forward_overlap must be in [0, 1)")
        if not 0.0 <= side_overlap < 1.0:
            raise ValueError("side_overlap must be in [0, 1)")
        if speed_mps <= 0 or turn_time_s < 0 or max_flight_time_s <= 0:
            raise ValueError("speed/time parameters must be positive")
        self.camera = camera
        self.target_gsd_m = float(target_gsd_m)
        self.forward_overlap = float(forward_overlap)
        self.side_overlap = float(side_overlap)
        self.speed_mps = float(speed_mps)
        self.turn_time_s = float(turn_time_s)
        self.max_flight_time_s = float(max_flight_time_s)
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)

    def plan(self, polygon: np.ndarray) -> SurveyPlan:
        polygon = np.asarray(polygon, dtype=float)
        if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
            raise ValueError("polygon must be an (N, 2) array with at least 3 vertices")

        angle = principal_angle(polygon)
        rotated = rotate_points(polygon, -angle)
        minimum = rotated.min(axis=0)
        maximum = rotated.max(axis=0)

        altitude = self.camera.altitude_for_gsd_m(self.target_gsd_m)
        footprint_width, footprint_height = self.camera.footprint_m(altitude)
        line_spacing = footprint_width * (1.0 - self.side_overlap)
        shot_spacing = footprint_height * (1.0 - self.forward_overlap)
        if line_spacing <= 0 or shot_spacing <= 0:
            raise ValueError("overlap leaves no spacing between scan lines or images")

        line_ys = np.arange(
            minimum[1] + footprint_width * 0.5,
            maximum[1],
            line_spacing,
        )
        if len(line_ys) == 0:
            line_ys = np.array([float(np.mean(minimum[1] + maximum[1]))])

        rotated_waypoints: list[tuple[float, float]] = []
        scan_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        image_count = 0
        reverse = False
        for y in line_ys:
            intervals = line_intervals(float(y), rotated)
            for start_x, end_x in intervals:
                if reverse:
                    start_x, end_x = end_x, start_x
                rotated_waypoints.append((start_x, float(y)))
                rotated_waypoints.append((end_x, float(y)))
                segment = rotate_points(
                    np.array([[start_x, float(y)], [end_x, float(y)]]), angle
                )
                scan_segments.append(
                    ((float(segment[0, 0]), float(segment[0, 1])),
                     (float(segment[1, 0]), float(segment[1, 1])))
                )
                length = abs(end_x - start_x)
                image_count += int(math.floor(length / shot_spacing)) + 1
                reverse = not reverse

        if len(rotated_waypoints) < 2:
            raise ValueError("polygon produced fewer than two survey waypoints")

        waypoints = rotate_points(np.asarray(rotated_waypoints), angle)
        distance = polyline_length(waypoints)
        turns = max(0, len(scan_segments) - 1)
        flight_time = distance / self.speed_mps + turns * self.turn_time_s
        coverage = sample_coverage(rotated, line_ys, footprint_width * 0.5, resolution_m=2.0)

        warnings = []
        if altitude > 120.0:
            warnings.append(
                f"altitude {altitude:.1f} m exceeds the 120 m regulatory limit; "
                "use a longer lens or accept a larger GSD"
            )
        if flight_time > self.max_flight_time_s:
            warnings.append(
                f"estimated flight time {flight_time:.0f} s exceeds "
                f"max_flight_time_s={self.max_flight_time_s:.0f} s"
            )
        if coverage < 0.90:
            warnings.append(f"sampled coverage is only {coverage:.1%}")

        return SurveyPlan(
            altitude_m=altitude,
            gsd_m=self.camera.gsd_m(altitude),
            footprint_width_m=footprint_width,
            footprint_height_m=footprint_height,
            line_spacing_m=line_spacing,
            shot_spacing_m=shot_spacing,
            line_count=len(line_ys),
            image_count=image_count,
            flight_distance_m=distance,
            flight_time_s=flight_time,
            coverage_ratio=coverage,
            waypoints_enu=waypoints,
            scan_segments=scan_segments,
            warnings=warnings,
            origin_lat=self.origin_lat,
            origin_lon=self.origin_lon,
        )
