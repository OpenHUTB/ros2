"""Run independent analytical motor heating scenarios and export metrics."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


DEFAULT_SCENARIOS = (
    {"name": "cool_cruise", "ambient_c": 20.0, "speed_mps": 0.55, "turn_rate": 0.0},
    {"name": "warm_turn", "ambient_c": 28.0, "speed_mps": 0.75, "turn_rate": 0.5},
    {"name": "hot_high_load", "ambient_c": 35.0, "speed_mps": 0.8, "turn_rate": 0.7},
)


def _simulate(scenario: dict[str, float | str], derate: bool, steps: int, dt: float) -> list[dict[str, float | str]]:
    ambient = float(scenario["ambient_c"])
    speed = float(scenario["speed_mps"])
    turn = float(scenario["turn_rate"])
    temperature = ambient
    scale = 1.0
    rows = []
    active = False
    for index in range(steps):
        current = 1.0 + 9.0 * speed * scale + 4.0 * abs(turn) * scale
        cooling = 0.18 + 0.08 * speed * scale
        slope = (0.1 * current**2 - cooling * (temperature - ambient)) / 12.0
        predicted = temperature + max(0.0, slope) * 5.0
        if derate:
            active = predicted >= 65.0 if not active else predicted > 60.0
            scale = max(0.25, min(1.0, 1.0 - 0.03 * (predicted - 60.0))) if active else 1.0
        temperature += dt * slope
        rows.append({
            "scenario": str(scenario["name"]),
            "mode": "derated" if derate else "fixed_speed",
            "time_s": (index + 1) * dt,
            "temperature_c": temperature,
            "speed_mps": speed * scale,
            "speed_scale": scale,
        })
    return rows


def evaluate(steps: int = 6000, dt: float = 0.01, output: str | Path = "output/thermal_evaluation") -> list[dict[str, float | int | str]]:
    if steps < 100 or dt <= 0:
        raise ValueError("steps must be at least 100 and dt must be positive")
    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, float | str]] = []
    summary: list[dict[str, float | int | str]] = []
    for scenario in DEFAULT_SCENARIOS:
        for derate in (False, True):
            values = _simulate(scenario, derate, steps, dt)
            rows.extend(values)
            temperatures = np.asarray([row["temperature_c"] for row in values])
            scales = np.asarray([row["speed_scale"] for row in values])
            summary.append({
                "scenario": scenario["name"],
                "mode": "derated" if derate else "fixed_speed",
                "ambient_c": scenario["ambient_c"],
                "peak_temperature_c": round(float(temperatures.max()), 3),
                "mean_speed_mps": round(float(np.mean([row["speed_mps"] for row in values])), 3),
                "derated_percent": round(float(np.mean(scales < 0.999) * 100), 2),
            })
    for filename, data in (("scenario_curves.csv", rows), ("scenario_summary.csv", summary)):
        with (destination / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(data[0]))
            writer.writeheader()
            writer.writerows(data)
    (destination / "scenario_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
