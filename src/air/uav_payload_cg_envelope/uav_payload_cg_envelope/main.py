"""Compute combined UAV center of gravity and check payload limits."""

from __future__ import annotations

import argparse
from importlib.resources import files
import json
import math
import time
from pathlib import Path
from typing import Any

DEFAULT_INPUT = files(__package__).joinpath("sample_data", "cg_profile.json")
MAX_ITEM_MASS_KG = 10000.0
MAX_TOTAL_MASS_KG = 10000.0


def _number(
    value: Any,
    name: str,
    errors: list[str],
    *,
    positive: bool = False,
    maximum: float | None = None,
) -> float:
    """Parse a finite number and append a structured error if invalid."""
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        errors.append(f"{name} must be finite")
        return 0.0

    number = float(value)
    if positive and number <= 0:
        errors.append(f"{name} must be positive")
    if maximum is not None and number > maximum:
        errors.append(f"{name} must not exceed {maximum:g}")
    return number


def _vector(
    value: Any, name: str, errors: list[str]
) -> tuple[float, float, float]:
    """Parse a three-dimensional finite vector."""
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(
            type(component) not in (int, float)
            or not math.isfinite(float(component))
            for component in value
        )
    ):
        errors.append(f"{name} must contain three finite numbers")
        return (0.0, 0.0, 0.0)
    return tuple(float(component) for component in value)


def check_cg(profile: Any) -> tuple[dict[str, Any], list[str]]:
    """Combine aircraft and payload masses and evaluate the CG envelope."""
    if not isinstance(profile, dict):
        return {}, ["profile must be a JSON object"]

    errors: list[str] = []
    empty = profile.get("empty", {})
    limits = profile.get("limits", {})
    items = profile.get("payloads")
    if not isinstance(empty, dict) or not isinstance(limits, dict):
        return {}, ["empty and limits must be objects"]

    mass = _number(
        empty.get("mass_kg"),
        "empty.mass_kg",
        errors,
        positive=True,
        maximum=MAX_ITEM_MASS_KG,
    )
    cg = _vector(empty.get("cg_m"), "empty.cg_m", errors)
    x_min = _number(limits.get("cg_x_min_m"), "limits.cg_x_min_m", errors)
    x_max = _number(limits.get("cg_x_max_m"), "limits.cg_x_max_m", errors)
    lateral_limit = _number(
        limits.get("lateral_max_m"),
        "limits.lateral_max_m",
        errors,
        positive=True,
    )
    if x_min >= x_max:
        errors.append("cg_x_min_m must be less than cg_x_max_m")

    if not isinstance(items, list):
        errors.append("payloads must be a list")
        items = []

    moments = [mass * component for component in cg]
    total_mass = mass
    rows: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            errors.append(f"payloads[{index}] must be an object")
            continue
        payload_mass = _number(
            item.get("mass_kg"),
            f"payloads[{index}].mass_kg",
            errors,
            positive=True,
            maximum=MAX_ITEM_MASS_KG,
        )
        position = _vector(
            item.get("cg_m"), f"payloads[{index}].cg_m", errors
        )
        total_mass += payload_mass
        for axis in range(3):
            moments[axis] += payload_mass * position[axis]
        rows.append(
            {
                "name": str(item.get("name", f"payload_{index}")),
                "mass_kg": payload_mass,
            }
        )

    if math.isfinite(total_mass) and total_mass > MAX_TOTAL_MASS_KG:
        errors.append(f"total_mass_kg must not exceed {MAX_TOTAL_MASS_KG:g}")
    if not math.isfinite(total_mass) or not all(
        math.isfinite(value) for value in moments
    ):
        errors.append("combined mass and moments must be finite")
    if errors:
        return {}, errors

    combined_cg = tuple(value / total_mass for value in moments)
    front_margin = combined_cg[0] - x_min
    aft_margin = x_max - combined_cg[0]
    warnings: list[str] = []
    feasible = (
        front_margin >= 0
        and aft_margin >= 0
        and abs(combined_cg[1]) <= lateral_limit
    )
    if feasible and min(front_margin, aft_margin) < 0.005:
        warnings.append("longitudinal CG margin below 5 mm")

    return (
        {
            "total_mass_kg": round(total_mass, 4),
            "cg_x_m": round(combined_cg[0], 5),
            "cg_y_m": round(combined_cg[1], 5),
            "cg_z_m": round(combined_cg[2], 5),
            "cg_z_m_is_informational": True,
            "front_margin_m": round(front_margin, 5),
            "aft_margin_m": round(aft_margin, 5),
            "lateral_abs_m": round(abs(combined_cg[1]), 5),
            "feasible": feasible,
            "warnings": warnings,
            "payloads": rows,
        },
        errors,
    )


def main(argv: list[str] | None = None) -> int:
    """Run the command-line payload CG check."""
    parser = argparse.ArgumentParser(
        description="Check UAV payload center of gravity"
    )
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)

    try:
        if hasattr(args.input, "read_text"):
            raw_profile = args.input.read_text(encoding="utf-8")
        else:
            raw_profile = Path(args.input).read_text(encoding="utf-8")
        profile = json.loads(raw_profile)
    except (OSError, TypeError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1

    result, errors = check_cg(profile)
    for key, value in result.items():
        if key != "payloads":
            print(f"{key}={value}")
    if args.frame_delay > 0:
        time.sleep(args.frame_delay)
    if result:
        print(
            "NOTE: cg_z_m is informational; only longitudinal and "
            "lateral limits are checked"
        )
    for error in errors:
        print("ERROR:", error)
    for warning in result.get("warnings", []):
        print("WARNING:", warning)
    if errors or not result.get("feasible", False):
        print("FAIL: payload CG is outside the configured envelope")
        return 1
    print("PASS: payload CG remains inside the configured envelope")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
