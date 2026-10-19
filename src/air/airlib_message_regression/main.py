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
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _vector(value: Any, length: int, field: str) -> List[str]:
    errors: List[str] = []
    if not isinstance(value, list) or len(value) != length:
        return [f"{field} must contain {length} numbers"]
    if not all(_finite_number(item) for item in value):
        errors.append(f"{field} contains a non-finite or non-numeric value")
    return errors


def validate_message(message: Dict[str, Any]) -> List[str]:
    """Return human-readable contract errors for one AirLib-like message."""
    errors: List[str] = []
    kind = message.get("type")
    if kind not in {"imu", "gps", "lidar", "odometry"}:
        return [f"unsupported message type: {kind!r}"]
    if not message.get("frame_id"):
        errors.append("frame_id must not be empty")
    if not isinstance(message.get("timestamp_ns"), int) or message["timestamp_ns"] < 0:
        errors.append("timestamp_ns must be a non-negative integer")

    if kind == "imu":
        errors += _vector(message.get("linear_acceleration"), 3, "linear_acceleration")
        errors += _vector(message.get("angular_velocity"), 3, "angular_velocity")
        errors += _vector(message.get("orientation"), 4, "orientation")
        orientation = message.get("orientation")
        if isinstance(orientation, list) and len(orientation) == 4:
            norm = math.sqrt(sum(float(value) ** 2 for value in orientation))
            if abs(norm - 1.0) > 1e-3:
                errors.append("orientation quaternion norm must be approximately 1")

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
        if message.get("status") not in {-1, 0, 1, 2}:
            errors.append("status must be one of -1, 0, 1, 2")

    elif kind == "lidar":
        fields = message.get("fields")
        if not isinstance(fields, list) or not fields:
            errors.append("fields must be a non-empty list")
        else:
            names = []
            for field in fields:
                if not isinstance(field, dict) or not {
                    "name", "offset", "datatype", "count"
                }.issubset(field):
                    errors.append("each LiDAR field needs name/offset/datatype/count")
                    continue
                names.extend([field["name"]] * int(field["count"]))
            if len(names) != 4:
                errors.append("sample expects x/y/z/intensity fields")
        point_step = message.get("point_step")
        if not isinstance(point_step, int) or point_step <= 0:
            errors.append("point_step must be a positive integer")
        points = message.get("points")
        if not isinstance(points, list) or len(points) % 4 != 0:
            errors.append("points must be a flat list with four values per point")
        elif not all(_finite_number(value) for value in points):
            errors.append("points contains a non-finite or non-numeric value")

    elif kind == "odometry":
        errors += _vector(message.get("position"), 3, "position")
        errors += _vector(message.get("orientation"), 4, "orientation")
        errors += _vector(message.get("linear_velocity"), 3, "linear_velocity")

    return errors


def validate_dataset(
    messages: Iterable[Dict[str, Any]], frame_delay: float = 0.0
) -> Tuple[int, List[str]]:
    failures: List[str] = []
    count = 0
    for count, message in enumerate(messages, start=1):
        errors = validate_message(message)
        label = message.get("type", "unknown")
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
        messages = payload["messages"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    if not messages:
        print("ERROR: messages must not be empty")
        return 1

    count, failures = validate_dataset(messages, max(0.0, args.frame_delay))
    if failures:
        print(f"FAIL: {len(failures)} contract errors across {count} messages")
        return 1
    print(f"PASS: validated {count} AirLib messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
