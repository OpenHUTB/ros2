"""Validate fixed-wing PX4/ArduPilot SITL configuration before launch."""

from __future__ import annotations

import argparse
from importlib.resources import files
import json
import math
import time
from typing import Any, Dict, List, Tuple


DEFAULT_INPUT = files(__package__).joinpath(
    "sample_data", "fixed_wing_profile.json"
)
REQUIRED_ACTUATORS = {
    "aileron_left",
    "aileron_right",
    "elevator",
    "rudder",
    "throttle",
}
REQUIRED_SENSORS = {"airspeed", "gps", "imu", "barometer"}


def _finite(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def validate_profile(profile: Any) -> Tuple[List[str], List[str]]:
    """Return errors and warnings for a fixed-wing SITL profile."""
    errors: List[str] = []
    warnings: List[str] = []
    if not isinstance(profile, dict):
        return ["profile must be a JSON object"], warnings

    firmware = profile.get("firmware")
    if not isinstance(firmware, dict):
        errors.append("firmware must be an object")
    else:
        if firmware.get("stack") not in {"px4", "ardupilot"}:
            errors.append("firmware.stack must be px4 or ardupilot")
        if not isinstance(firmware.get("version"), str) or not firmware["version"]:
            errors.append("firmware.version must be a non-empty string")
        if type(firmware.get("sitl")) is not bool or not firmware["sitl"]:
            errors.append("firmware.sitl must be true")
        frame = firmware.get("frame")
        if not isinstance(frame, str) or "fixed" not in frame.lower():
            errors.append("firmware.frame must describe a fixed-wing airframe")

    connection = profile.get("connection")
    if not isinstance(connection, dict):
        errors.append("connection must be an object")
    else:
        host = connection.get("host")
        if not isinstance(host, str) or not host.strip():
            errors.append("connection.host must be a non-empty string")
        ports = {}
        for name in ("sitl_port", "mavlink_port"):
            value = connection.get(name)
            if type(value) is not int or not 1024 <= value <= 65535:
                errors.append(f"connection.{name} must be a valid TCP/UDP port")
            else:
                ports[name] = value
        if len(ports) == 2 and ports["sitl_port"] == ports["mavlink_port"]:
            errors.append("SITL and MAVLink ports must differ")

    origin = profile.get("origin")
    if not isinstance(origin, dict):
        errors.append("origin must be an object")
    else:
        latitude = origin.get("latitude_deg")
        longitude = origin.get("longitude_deg")
        altitude = origin.get("altitude_m")
        if not _finite(latitude) or not -90 <= float(latitude) <= 90:
            errors.append("origin.latitude_deg must be within [-90, 90]")
        if not _finite(longitude) or not -180 <= float(longitude) <= 180:
            errors.append("origin.longitude_deg must be within [-180, 180]")
        if not _finite(altitude) or float(altitude) < -500:
            errors.append("origin.altitude_m must be a plausible finite altitude")

    actuators = profile.get("actuators")
    if not isinstance(actuators, dict):
        errors.append("actuators must be an object")
    else:
        missing = sorted(REQUIRED_ACTUATORS - set(actuators))
        if missing:
            errors.append(f"missing required fixed-wing actuators: {missing}")
        for name, channel in actuators.items():
            if type(channel) is not int or not 1 <= channel <= 16:
                errors.append(f"actuator {name} must use channel 1..16")
        valid_channels = [
            value
            for value in actuators.values()
            if type(value) is int and 1 <= value <= 16
        ]
        if len(valid_channels) != len(set(valid_channels)):
            errors.append("actuator channels must be unique")

    sensors = profile.get("sensors")
    if not isinstance(sensors, dict):
        errors.append("sensors must be an object")
    else:
        for name in sorted(REQUIRED_SENSORS):
            if sensors.get(name) is not True:
                errors.append(f"fixed-wing SITL requires sensors.{name}=true")

    failsafe = profile.get("failsafe")
    if not isinstance(failsafe, dict):
        errors.append("failsafe must be an object")
    else:
        for name in ("rc_loss", "data_link_loss"):
            if failsafe.get(name) not in {"return", "loiter", "hold"}:
                errors.append(f"failsafe.{name} must be return, loiter or hold")
        minimum_battery = failsafe.get("minimum_battery_percent")
        if type(minimum_battery) is not int or not 10 <= minimum_battery <= 50:
            errors.append("failsafe.minimum_battery_percent must be 10..50")
        elif minimum_battery < 20:
            warnings.append("battery reserve below 20 percent increases risk")

    return errors, warnings


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate a fixed-wing PX4/ArduPilot SITL profile"
    )
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)

    try:
        profile = json.loads(open(args.input, "r", encoding="utf-8").read())
    except (OSError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1

    errors, warnings = validate_profile(profile)
    print("fixed-wing SITL preflight")
    print(f"errors={len(errors)} warnings={len(warnings)}")
    if args.frame_delay > 0:
        time.sleep(args.frame_delay)
    for item in errors:
        print(f"ERROR: {item}")
    for item in warnings:
        print(f"WARNING: {item}")
    if errors:
        print("FAIL: configuration is not ready for fixed-wing SITL")
        return 1
    print("PASS: fixed-wing SITL configuration is internally consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
