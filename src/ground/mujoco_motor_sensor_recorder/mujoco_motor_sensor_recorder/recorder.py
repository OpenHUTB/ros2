"""Apply deterministic sensor noise to a standalone motor-state CSV."""

from __future__ import annotations

import csv
import random
from pathlib import Path

INPUT_FIELDS = (
    "time_s", "speed_mps", "left_temperature_c", "right_temperature_c",
    "left_torque_nm", "right_torque_nm", "ambient_c",
)
OUTPUT_FIELDS = (
    "time_s", "speed_mps", "left_temperature_c", "right_temperature_c",
    "left_current_a", "right_current_a", "ambient_c",
)


def record_csv(
    input_path: str | Path,
    output_path: str | Path,
    *,
    seed: int = 7,
    temperature_noise_c: float = 0.12,
    current_noise_a: float = 0.04,
    torque_constant_nm_per_a: float = 0.18,
    idle_current_a: float = 0.25,
) -> int:
    if min(temperature_noise_c, current_noise_a) < 0 or torque_constant_nm_per_a <= 0 or idle_current_a < 0:
        raise ValueError("noise and idle current must be non-negative; torque constant must be positive")
    rng = random.Random(seed)
    input_file = Path(input_path)
    output = Path(output_path)
    if input_file.resolve() == output.resolve():
        raise ValueError("input_path and output_path must be different")
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with input_file.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames or not set(INPUT_FIELDS).issubset(reader.fieldnames):
            raise ValueError(f"input CSV must contain: {', '.join(INPUT_FIELDS)}")
        with output.open("w", newline="", encoding="utf-8") as destination:
            writer = csv.DictWriter(destination, fieldnames=OUTPUT_FIELDS)
            writer.writeheader()
            for row in reader:
                left_torque = float(row["left_torque_nm"])
                right_torque = float(row["right_torque_nm"])
                observations = {
                    "time_s": float(row["time_s"]),
                    "speed_mps": float(row["speed_mps"]),
                    "left_temperature_c": float(row["left_temperature_c"]) + rng.gauss(0, temperature_noise_c),
                    "right_temperature_c": float(row["right_temperature_c"]) + rng.gauss(0, temperature_noise_c),
                    "left_current_a": max(0.0, abs(left_torque) / torque_constant_nm_per_a + idle_current_a + rng.gauss(0, current_noise_a)),
                    "right_current_a": max(0.0, abs(right_torque) / torque_constant_nm_per_a + idle_current_a + rng.gauss(0, current_noise_a)),
                    "ambient_c": float(row["ambient_c"]),
                }
                # CSV precision is part of the output contract: suppress
                # insignificant libm differences in random.gauss across OSes.
                writer.writerow({field: f"{value:.6f}" for field, value in observations.items()})
                count += 1
    return count
