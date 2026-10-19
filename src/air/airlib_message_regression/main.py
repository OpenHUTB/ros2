"""Validate AirLib sensor payloads before ROS bridge ingestion."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple


DEFAULT_INPUT = Path(__file__).resolve().parent / "sample_data" / "sample_messages.json"


def _finite_number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _vector(value: Any, length: int, field: str) -> List[str]:
    if not isinstance(value, list) or len(value) != length:
        return [f"{field} must contain {length} numbers"]
    if not all(_finite_number(item) for item in value):
        return [f"{field} contains a non-finite or non-numeric value"]
    return []


def _quaternion(value: Any, field: str) -> List[str]:
    errors = _vector(value, 4, field)
    if errors:
        return errors
    norm = math.sqrt(sum(float(item) ** 2 for item in value))
    if abs(norm - 1.0) > 1e-3:
        return [f"{field} quaternion norm must be approximately 1"]
    return []


def validate_message(message: Any) -> List[str]:
    """Return human-readable contract errors for one AirLib-like message."""
    if not isinstance(message, dict):
        return ["message must be a JSON object"]
    errors: List[str] = []
    kind = message.get("type")
    if kind not in {"imu", "gps", "lidar", "odometry"}:
        return [f"unsupported message type: {kind!r}"]
    if not message.get("frame_id"):
        errors.append("frame_id must not be empty")
    timestamp_ns = message.get("timestamp_ns")
    if type(timestamp_ns) is not int or timestamp_ns < 0:
        errors.append("timestamp_ns must be a non-negative integer")

    if kind == "imu":
        errors += _vector(message.get("linear_acceleration"), 3, "linear_acceleration")
        errors += _vector(message.get("angular_velocity"), 3, "angular_velocity")
        errors += _quaternion(message.get("orientation"), "orientation")

    elif kind == "gps":
        latitude = message.get("latitude")
        longitude = message.get("longitude")
        altitude = message.get("altitude")
        if not _finite_number(latitude) or not -90.0 <= float(latitude) <= 90.0:
            errors.append("latitude must be within [-90, 90]")
        if not _finite_number(longitude) or not -180.0 <= float(longitude) <= 180.0:
            errors.append("longitude must be within [-180, 180]")
        if not _finite_number(altitude):
            errors.append("altitude must be finite")
        status = message.get("status")
        if type(status) is not int or status not in {-1, 0, 1, 2}:
            errors.append("status must be one of -1, 0, 1, 2")

    elif kind == "lidar":
        fields = message.get("fields")
        if not isinstance(fields, list) or not fields:
            errors.append("fields must be a non-empty list")
        else:
            names: List[str] = []
            for field in fields:
                if not isinstance(field, dict) or not {
                    "name", "offset", "datatype", "count"
                }.issubset(field):
                    errors.append("each LiDAR field needs name/offset/datatype/count")
                    continue
                name = field["name"]
                offset = field["offset"]
                datatype = field["datatype"]
                count = field["count"]
                if not isinstance(name, str) or not name:
                    errors.append("LiDAR field name must be a non-empty string")
                    continue
                if type(offset) is not int or offset < 0:
                    errors.append(f"LiDAR field {name} offset must be a non-negative integer")
                    continue
                if type(datatype) is not int or datatype < 0:
                    errors.append(f"LiDAR field {name} datatype must be a non-negative integer")
                    continue
                if type(count) is not int or count <= 0:
                    errors.append(f"LiDAR field {name} count must be a positive integer")
                    continue
                names.extend([name] * count)
            if len(names) != 4 or set(names) != {"x", "y", "z", "intensity"}:
                errors.append("LiDAR fields must be x/y/z/intensity")
        point_step = message.get("point_step")
        if type(point_step) is not int or point_step <= 0:
            errors.append("point_step must be a positive integer")
        points = message.get("points")
        if not isinstance(points, list) or len(points) % 4 != 0:
            errors.append("points must be a flat list with four values per point")
        elif not all(_finite_number(value) for value in points):
            errors.append("points contains a non-finite or non-numeric value")

    elif kind == "odometry":
        errors += _vector(message.get("position"), 3, "position")
        errors += _quaternion(message.get("orientation"), "orientation")
        errors += _vector(message.get("linear_velocity"), 3, "linear_velocity")

    return errors


def validate_dataset(
    messages: Iterable[Dict[str, Any]], frame_delay: float = 0.0
) -> Tuple[int, List[str]]:
    failures: List[str] = []
    count = 0
    for count, message in enumerate(messages, start=1):
        errors = validate_message(message)
        label = message.get("type", "unknown") if isinstance(message, dict) else "unknown"
        status = "PASS" if not errors else "FAIL"
        print(f"{count:02d} {label:<10} {status}")
        for error in errors:
            print(f"   - {error}")
            failures.append(f"message {count}: {error}")
        if frame_delay > 0:
            time.sleep(frame_delay)
    return count, failures


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate AirLib IMU/GPS/LiDAR/odometry message contracts"
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)

    try:
        payload = json.loads(args.input.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            print("ERROR: top-level JSON value must be an object")
            return 1
        messages = payload["messages"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    if not isinstance(messages, list) or not messages:
        print("ERROR: messages must be a non-empty list")
        return 1

    count, failures = validate_dataset(messages, max(0.0, args.frame_delay))
    if failures:
        print(f"FAIL: {len(failures)} contract errors across {count} messages")
        return 1
    print(f"PASS: validated {count} AirLib messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
