#!/usr/bin/env python3
"""Summarize a DReyeVR driving-study session with the Python standard library.

The recorder documented in ``Docs/driving_study.md`` writes two CSV files:

* ``<session>.csv``      -- one row per game-thread sample (71 columns)
* ``<session>_gaze.csv`` -- one row per eye-tracker sample (18 columns)

This tool deliberately has no pandas/numpy dependency.  It is intended both as
a reusable analysis module and as a small, reviewable first PR.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


DRIVING_REQUIRED_COLUMNS = {
    "schema_version",
    "ue_frame",
    "sim_time_s",
    "speed_kmh",
    "steer_input",
    "throttle_input",
    "brake_input",
    "gaze_valid",
    "looking_at_affecting_light",
    "red_light_stop_candidate",
}

GAZE_REQUIRED_COLUMNS = {
    "sim_time_s",
    "valid",
    "saccade_state",
    "saccade_onset_s",
    "saccade_amp_deg",
}


class SchemaError(ValueError):
    """Raised when a CSV does not match the documented recorder schema."""


def _read_csv(path: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    if not path.is_file():
        raise FileNotFoundError("CSV not found: {}".format(path))
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        columns = list(reader.fieldnames or [])
        rows = [dict(row) for row in reader]
    return rows, columns


def _require_columns(columns: Sequence[str], required: Iterable[str], label: str) -> None:
    missing = sorted(set(required) - set(columns))
    if missing:
        raise SchemaError("{} CSV is missing columns: {}".format(label, ", ".join(missing)))


def _number(value: Optional[str], default: Optional[float] = None) -> Optional[float]:
    if value is None:
        return default
    text = str(value).strip()
    if not text:
        return default
    try:
        result = float(text)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _quantile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _ratio(rows: Sequence[Dict[str, str]], predicate) -> Optional[float]:
    if not rows:
        return None
    return sum(1 for row in rows if predicate(row)) / float(len(rows))


def _nearest_delta(sorted_times: Sequence[float], value: float) -> Optional[float]:
    if not sorted_times:
        return None
    index = bisect.bisect_left(sorted_times, value)
    candidates = []
    if index < len(sorted_times):
        candidates.append(abs(sorted_times[index] - value))
    if index > 0:
        candidates.append(abs(sorted_times[index - 1] - value))
    return min(candidates) if candidates else None


def summarize(
    driving_rows: Sequence[Dict[str, str]],
    gaze_rows: Sequence[Dict[str, str]],
    driving_columns: Optional[Sequence[str]] = None,
    gaze_columns: Optional[Sequence[str]] = None,
) -> Dict[str, object]:
    """Return a JSON-serialisable summary of one driving-study session."""

    warnings: List[str] = []
    if driving_columns is not None:
        _require_columns(driving_columns, DRIVING_REQUIRED_COLUMNS, "driving")
    if gaze_columns is not None and gaze_rows:
        _require_columns(gaze_columns, GAZE_REQUIRED_COLUMNS, "gaze")

    driving: List[Tuple[Dict[str, str], float]] = []
    for row in driving_rows:
        timestamp = _number(row.get("sim_time_s"))
        if timestamp is not None:
            driving.append((row, timestamp))
    if not driving:
        raise SchemaError("driving CSV has no rows with a valid sim_time_s")

    gaze: List[Tuple[Dict[str, str], float]] = []
    for row in gaze_rows:
        timestamp = _number(row.get("sim_time_s"))
        if timestamp is not None:
            gaze.append((row, timestamp))
    if gaze_rows and not gaze:
        warnings.append("gaze CSV has no rows with a valid sim_time_s")

    driving_times = [timestamp for _, timestamp in driving]
    duration = max(driving_times) - min(driving_times)
    sample_rate = None
    if duration > 0.0:
        sample_rate = (len(driving_times) - 1) / duration

    speeds = [
        value
        for row, _ in driving
        for value in [_number(row.get("speed_kmh"))]
        if value is not None
    ]

    def active(row: Dict[str, str], key: str) -> bool:
        value = _number(row.get(key), 0.0)
        return value is not None and abs(value) > 0.05

    valid_gaze_rows = [row for row, _ in driving if row.get("gaze_valid") == "1"]
    gaze_states = [
        (row.get("saccade_state") or "").strip().lower()
        for row, _ in gaze
        if (row.get("saccade_state") or "").strip()
    ]
    state_counts: Dict[str, int] = {}
    for state in gaze_states:
        state_counts[state] = state_counts.get(state, 0) + 1

    # One saccade event is repeated on every sample of the event.  Count unique
    # onset timestamps instead of counting rows.
    saccade_amplitudes: Dict[float, Optional[float]] = {}
    for row, timestamp in gaze:
        if (row.get("saccade_state") or "").strip().lower() != "saccade":
            continue
        onset = _number(row.get("saccade_onset_s"))
        key = onset if onset is not None else round(timestamp, 3)
        saccade_amplitudes[key] = _number(row.get("saccade_amp_deg"))

    gaze_times = sorted(timestamp for _, timestamp in gaze)
    alignment_deltas = [
        delta
        for delta in (_nearest_delta(gaze_times, timestamp) for timestamp in driving_times)
        if delta is not None
    ]

    def count_non_empty(key: str) -> int:
        return sum(1 for row, _ in driving if (row.get(key) or "").strip())

    report: Dict[str, object] = {
        "schema_version": driving[0][0].get("schema_version"),
        "rows": {
            "driving": len(driving_rows),
            "driving_with_time": len(driving),
            "gaze": len(gaze_rows),
            "gaze_with_time": len(gaze),
        },
        "time": {
            "start_s": min(driving_times),
            "end_s": max(driving_times),
            "duration_s": duration,
            "sample_rate_hz": sample_rate,
        },
        "speed_kmh": {
            "mean": statistics.mean(speeds) if speeds else None,
            "max": max(speeds) if speeds else None,
            "p95": _quantile(speeds, 0.95),
        },
        "controls": {
            "steer_active_ratio": _ratio([row for row, _ in driving], lambda row: active(row, "steer_input")),
            "throttle_active_ratio": _ratio([row for row, _ in driving], lambda row: active(row, "throttle_input")),
            "brake_active_ratio": _ratio([row for row, _ in driving], lambda row: active(row, "brake_input")),
        },
        "gaze": {
            "valid_ratio_driving": _ratio([row for row, _ in driving], lambda row: row.get("gaze_valid") == "1"),
            "looking_at_affecting_light_ratio": _ratio(
                valid_gaze_rows, lambda row: row.get("looking_at_affecting_light") == "1"
            ),
            "gaze_state_counts": state_counts,
            "fixation_ratio": (
                state_counts.get("fixation", 0) / float(sum(state_counts.values()))
                if sum(state_counts.values())
                else None
            ),
            "saccade_event_count": len(saccade_amplitudes),
            "mean_saccade_amplitude_deg": (
                statistics.mean([value for value in saccade_amplitudes.values() if value is not None])
                if any(value is not None for value in saccade_amplitudes.values())
                else None
            ),
        },
        "events": {
            "red_light_stop_candidate_rows": sum(
                1 for row, _ in driving if row.get("red_light_stop_candidate") == "1"
            ),
            "non_empty_traffic_light_rows": count_non_empty("traffic_light_id"),
        },
        "alignment": {
            "median_nearest_gaze_delta_ms": (
                statistics.median(alignment_deltas) * 1000.0 if alignment_deltas else None
            ),
            "max_nearest_gaze_delta_ms": (max(alignment_deltas) * 1000.0 if alignment_deltas else None),
        },
        "warnings": warnings,
    }
    return report


def analyze_files(driving_path: Path, gaze_path: Optional[Path]) -> Dict[str, object]:
    driving_rows, driving_columns = _read_csv(driving_path)
    if gaze_path is None:
        gaze_rows: List[Dict[str, str]] = []
        gaze_columns: Optional[List[str]] = None
    else:
        gaze_rows, gaze_columns = _read_csv(gaze_path)
    return summarize(driving_rows, gaze_rows, driving_columns, gaze_columns)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--driving", type=Path, required=True, help="<session>.csv")
    parser.add_argument("--gaze", type=Path, help="<session>_gaze.csv")
    parser.add_argument("--out", type=Path, default=Path("driving_study_report.json"))
    args = parser.parse_args(argv)

    report = analyze_files(args.driving, args.gaze)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
