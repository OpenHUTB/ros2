"""Solve UAV heading and ground speed from wind and desired course."""

from __future__ import annotations

import argparse
from importlib.resources import files
import json
import math
import time
from typing import Any, Dict, List


DEFAULT_INPUT = files(__package__).joinpath("sample_data", "wind_profile.json")


def _wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def solve_wind_triangle(profile: Any) -> tuple[Dict[str, float], List[str]]:
    errors: List[str] = []
    if not isinstance(profile, dict):
        return {}, ["profile must be a JSON object"]
    values = {}
    for name in ("true_airspeed_ms", "desired_course_deg", "wind_speed_ms", "wind_from_deg"):
        value = profile.get(name)
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            errors.append(f"{name} must be finite")
        else:
            values[name] = float(value)
    if errors:
        return {}, errors
    airspeed = values["true_airspeed_ms"]
    wind_speed = values["wind_speed_ms"]
    if airspeed <= 0:
        errors.append("true_airspeed_ms must be positive")
    if wind_speed < 0:
        errors.append("wind_speed_ms must be non-negative")
    if errors:
        return {}, errors

    course = math.radians(values["desired_course_deg"] % 360.0)
    wind_to = math.radians((values["wind_from_deg"] + 180.0) % 360.0)
    sin_c, cos_c = math.sin(course), math.cos(course)
    wind_x = wind_speed * math.sin(wind_to)
    wind_y = wind_speed * math.cos(wind_to)
    wind_lateral = wind_x * cos_c - wind_y * sin_c
    wind_forward = wind_x * sin_c + wind_y * cos_c
    air_lateral = -wind_lateral
    if abs(air_lateral) > airspeed:
        return {}, ["crosswind component exceeds true airspeed; course is unreachable"]
    air_forward = math.sqrt(max(0.0, airspeed ** 2 - air_lateral ** 2))
    air_x = air_forward * sin_c + air_lateral * cos_c
    air_y = air_forward * cos_c - air_lateral * sin_c
    heading = math.degrees(math.atan2(air_x, air_y)) % 360.0
    ground_speed = air_forward + wind_forward
    if ground_speed <= 0:
        return {}, ["headwind prevents forward progress along the desired course"]
    return {
        "required_heading_deg": round(heading, 3),
        "ground_speed_ms": round(ground_speed, 3),
        "wind_correction_deg": round(_wrap_deg(heading - values["desired_course_deg"]), 3),
        "headwind_component_ms": round(-wind_forward, 3),
        "crosswind_component_ms": round(wind_lateral, 3),
    }, errors


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Solve a UAV wind triangle")
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)
    try:
        profile = json.loads(open(args.input, "r", encoding="utf-8").read())
    except (OSError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    result, errors = solve_wind_triangle(profile)
    for key, value in result.items():
        print(f"{key}={value}")
        if args.frame_delay > 0:
            time.sleep(args.frame_delay)
    for item in errors:
        print(f"ERROR: {item}")
    if errors:
        print("FAIL: wind triangle has no safe solution")
        return 1
    print("PASS: heading and ground speed solved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
