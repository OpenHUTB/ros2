"""Estimate multirotor mission energy and battery reserve."""

from __future__ import annotations

import argparse
from importlib.resources import files
import json
import math
import time
from typing import Any, Dict, List, Tuple


DEFAULT_INPUT = files(__package__).joinpath("sample_data", "mission_profile.json")


def _number(value: Any, name: str, errors: List[str], *, positive=False) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        errors.append(f"{name} must be finite")
        return 0.0
    number = float(value)
    if positive and number <= 0:
        errors.append(f"{name} must be positive")
    return number


def evaluate_mission(profile: Any) -> Tuple[List[Dict[str, Any]], Dict[str, Any], List[str]]:
    """Return segment results, mission summary and validation errors."""
    errors: List[str] = []
    rows: List[Dict[str, Any]] = []
    summary: Dict[str, Any] = {}
    if not isinstance(profile, dict):
        return rows, summary, ["profile must be a JSON object"]

    battery = profile.get("battery", {})
    aircraft = profile.get("aircraft", {})
    if not isinstance(battery, dict) or not isinstance(aircraft, dict):
        return rows, summary, ["battery and aircraft must be objects"]

    capacity_wh = _number(battery.get("capacity_wh"), "battery.capacity_wh", errors, positive=True)
    reserve_fraction = _number(battery.get("reserve_fraction"), "battery.reserve_fraction", errors)
    efficiency = _number(battery.get("efficiency"), "battery.efficiency", errors, positive=True)
    empty_mass = _number(aircraft.get("empty_mass_kg"), "aircraft.empty_mass_kg", errors, positive=True)
    payload_mass = _number(aircraft.get("payload_kg"), "aircraft.payload_kg", errors)
    mass_exponent = _number(aircraft.get("mass_power_exponent"), "aircraft.mass_power_exponent", errors, positive=True)
    hover_power = _number(aircraft.get("hover_power_w"), "aircraft.hover_power_w", errors, positive=True)
    climb_extra = _number(aircraft.get("climb_extra_power_w"), "aircraft.climb_extra_power_w", errors)
    climb_rate = _number(aircraft.get("climb_rate_ms"), "aircraft.climb_rate_ms", errors, positive=True)
    cruise_power = _number(aircraft.get("cruise_power_w"), "aircraft.cruise_power_w", errors, positive=True)
    cruise_airspeed = _number(aircraft.get("cruise_airspeed_ms"), "aircraft.cruise_airspeed_ms", errors, positive=True)
    headwind = _number(profile.get("headwind_ms", 0.0), "headwind_ms", errors)

    if not 0 <= reserve_fraction < 1:
        errors.append("battery.reserve_fraction must be in [0, 1)")
    if not 0 < efficiency <= 1:
        errors.append("battery.efficiency must be in (0, 1]")
    if payload_mass < 0:
        errors.append("aircraft.payload_kg must be non-negative")
    if climb_extra < 0:
        errors.append("aircraft.climb_extra_power_w must be non-negative")
    ground_speed = cruise_airspeed - headwind
    if ground_speed <= 0:
        errors.append("headwind must be lower than cruise_airspeed_ms")

    if errors:
        return rows, summary, errors

    mass_factor = ((empty_mass + payload_mass) / empty_mass) ** mass_exponent
    usable_wh = capacity_wh * efficiency
    reserve_wh = capacity_wh * reserve_fraction
    segments = profile.get("segments")
    if not isinstance(segments, list) or not segments:
        return rows, summary, ["segments must be a non-empty list"]

    total_energy = 0.0
    total_distance = 0.0
    total_duration = 0.0
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            errors.append(f"segments[{index}] must be an object")
            continue
        kind = segment.get("kind")
        name = str(segment.get("name", f"segment_{index}"))
        power = hover_power * mass_factor
        duration_s = 0.0
        distance_m = 0.0
        altitude_m = 0.0
        if kind == "hover":
            duration_s = _number(segment.get("duration_s"), f"{name}.duration_s", errors, positive=True)
        elif kind == "climb":
            altitude_m = _number(segment.get("altitude_m"), f"{name}.altitude_m", errors, positive=True)
            duration_s = altitude_m / climb_rate
            power += climb_extra
        elif kind == "cruise":
            distance_m = _number(segment.get("distance_m"), f"{name}.distance_m", errors, positive=True)
            duration_s = distance_m / ground_speed
            power = cruise_power * mass_factor
        else:
            errors.append(f"{name}.kind must be hover, climb or cruise")
            continue
        energy_wh = power * duration_s / 3600.0
        rows.append({"name": name, "kind": kind, "duration_s": round(duration_s, 3), "distance_m": round(distance_m, 3), "energy_wh": round(energy_wh, 4)})
        total_energy += energy_wh
        total_distance += distance_m
        total_duration += duration_s

    if errors:
        return rows, summary, errors
    margin_wh = usable_wh - reserve_wh - total_energy
    remaining_percent = (usable_wh - total_energy) / capacity_wh * 100.0
    max_cruise_distance = max(0.0, (usable_wh - reserve_wh) / cruise_power * 3600.0 * ground_speed)
    summary = {
        "total_energy_wh": round(total_energy, 4),
        "usable_energy_wh": round(usable_wh, 4),
        "reserve_energy_wh": round(reserve_wh, 4),
        "remaining_percent": round(remaining_percent, 3),
        "margin_wh": round(margin_wh, 4),
        "mission_duration_s": round(total_duration, 3),
        "mission_distance_m": round(total_distance, 3),
        "max_cruise_distance_m": round(max_cruise_distance, 3),
        "feasible": margin_wh >= 0,
    }
    return rows, summary, errors


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Estimate UAV mission energy and endurance")
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)
    try:
        profile = json.loads(open(args.input, "r", encoding="utf-8").read())
    except (OSError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    rows, summary, errors = evaluate_mission(profile)
    print("segment  kind    duration_s  distance_m  energy_wh")
    for row in rows:
        print(f"{row['name']:<12} {row['kind']:<7} {row['duration_s']:>10.2f} {row['distance_m']:>11.2f} {row['energy_wh']:>10.3f}")
        if args.frame_delay > 0:
            time.sleep(args.frame_delay)
    for item in errors:
        print(f"ERROR: {item}")
    if errors:
        print("FAIL: mission profile is invalid")
        return 1
    print(f"total_energy_wh={summary['total_energy_wh']:.3f}")
    print(f"remaining_percent={summary['remaining_percent']:.1f}")
    print(f"margin_wh={summary['margin_wh']:.3f}")
    print(f"max_cruise_distance_m={summary['max_cruise_distance_m']:.1f}")
    if not summary["feasible"]:
        print("FAIL: battery reserve is insufficient")
        return 1
    print("PASS: mission fits within usable battery and reserve")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
