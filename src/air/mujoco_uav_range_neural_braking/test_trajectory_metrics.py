"""轨迹指标分析测试。Tests for trajectory metric analysis."""

import csv
import json
from pathlib import Path
import subprocess
import sys

import pytest

from trajectory_metrics import summarize_trajectory


FIELDNAMES = ["time_s", "range_m", "speed_m_s", "accel_m_s2", "collision", "sensor_valid"]


def write_trajectory(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def test_summarize_trajectory_reports_control_metrics(tmp_path):
    path = tmp_path / "trajectory.csv"
    write_trajectory(path, [
        {"time_s": "0", "range_m": "4.0", "speed_m_s": "1.5", "accel_m_s2": "-2.5", "collision": "0", "sensor_valid": "1"},
        {"time_s": "1", "range_m": "2.5", "speed_m_s": "0.4", "accel_m_s2": "-1.0", "collision": "0", "sensor_valid": "1"},
        {"time_s": "2", "range_m": "2.0", "speed_m_s": "0.03", "accel_m_s2": "0.2", "collision": "1", "sensor_valid": "0"},
        {"time_s": "3", "range_m": "2.1", "speed_m_s": "0.01", "accel_m_s2": "0.0", "collision": "0", "sensor_valid": "1"},
    ])

    metrics = summarize_trajectory(path)

    assert metrics["samples"] == 4
    assert metrics["initial_clearance_m"] == pytest.approx(4.0)
    assert metrics["min_clearance_m"] == pytest.approx(2.0)
    assert metrics["braking_distance_m"] == pytest.approx(2.0)
    assert metrics["overshoot_m"] == 0.0
    assert metrics["peak_abs_accel_m_s2"] == pytest.approx(2.5)
    assert metrics["settling_time_s"] == pytest.approx(2.0)
    assert metrics["final_speed_m_s"] == pytest.approx(0.01)
    assert metrics["collision_steps"] == 1
    assert metrics["invalid_sensor_steps"] == 1


def test_summarize_trajectory_rejects_empty_or_incomplete_input(tmp_path):
    empty = tmp_path / "empty.csv"
    empty.write_text(",".join(FIELDNAMES) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="为空"):
        summarize_trajectory(empty)

    incomplete = tmp_path / "incomplete.csv"
    incomplete.write_text("time_s,range_m\n0,1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="缺少轨迹字段"):
        summarize_trajectory(incomplete)

    for column, invalid_value in (("collision", "0.5"), ("sensor_valid", "inf")):
        invalid = tmp_path / f"invalid_{column}.csv"
        row = {
            "time_s": "0", "range_m": "1", "speed_m_s": "0",
            "accel_m_s2": "0", "collision": "0", "sensor_valid": "1",
        }
        row[column] = invalid_value
        write_trajectory(invalid, [row])
        with pytest.raises(ValueError, match="字段 .* 必须为有限值|字段 .* 必须为 0 或 1"):
            summarize_trajectory(invalid)


def test_cli_writes_json_metrics(tmp_path):
    path = tmp_path / "trajectory.csv"
    write_trajectory(path, [
        {"time_s": "0", "range_m": "2.0", "speed_m_s": "0.4", "accel_m_s2": "-1", "collision": "0", "sensor_valid": "1"},
        {"time_s": "1", "range_m": "1.8", "speed_m_s": "0.01", "accel_m_s2": "0", "collision": "0", "sensor_valid": "1"},
    ])
    output = tmp_path / "metrics.json"
    script = Path(__file__).with_name("trajectory_metrics.py")

    result = subprocess.run(
        [sys.executable, str(script), str(path), "--output", str(output)],
        capture_output=True, text=True, encoding="utf-8",
    )

    assert result.returncode == 0
    assert json.loads(output.read_text(encoding="utf-8"))["braking_distance_m"] == pytest.approx(0.2)
