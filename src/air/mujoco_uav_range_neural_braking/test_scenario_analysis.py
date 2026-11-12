"""参数分组统计测试。Tests for grouped scenario statistics."""

import csv
import json
from pathlib import Path
import subprocess
import sys

import pytest

from scenario_analysis import analyze_sweep_csv


FIELDS = [
    "scenario", "controller", "start_x_m", "wall_x_m", "initial_speed_m_s", "height_m",
    "steps", "collision_steps", "min_clearance_m", "final_speed_m_s",
    "invalid_sensor_steps", "max_height_error_m",
]


def write_sweep_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def make_row(scenario, speed, clearance, collision_steps):
    return {
        "scenario": scenario, "controller": "range", "start_x_m": -8,
        "wall_x_m": 0, "initial_speed_m_s": speed, "height_m": 1.2,
        "steps": 1000, "collision_steps": collision_steps,
        "min_clearance_m": clearance, "final_speed_m_s": 0,
        "invalid_sensor_steps": 0, "max_height_error_m": 0,
    }


def test_analysis_reports_grouped_clearance_and_collision_rate(tmp_path):
    path = tmp_path / "sweep.csv"
    write_sweep_csv(path, [
        make_row("safe_a", 0, 0.8, 0),
        make_row("safe_b", 0, 0.6, 0),
        make_row("contact_a", 1.5, -0.1, 2),
        make_row("contact_b", 1.5, -0.3, 4),
    ])

    report = analyze_sweep_csv(path)

    assert report["scenario_count"] == 4
    assert report["collision_scenarios"] == 2
    assert report["collision_rate"] == pytest.approx(0.5)
    assert report["minimum_clearance_m"] == pytest.approx(-0.3)
    assert report["worst_scenario"] == "contact_b"
    speed_groups = report["factor_summaries"]["initial_speed_m_s"]
    assert speed_groups[0]["mean_min_clearance_m"] == pytest.approx(0.7)
    assert speed_groups[1]["worst_min_clearance_m"] == pytest.approx(-0.3)
    assert speed_groups[1]["collision_rate"] == pytest.approx(1.0)
    assert report["factor_ranking"][0]["parameter"] == "initial_speed_m_s"


@pytest.mark.parametrize("contents, message", [
    ("scenario,min_clearance_m\na,1\n", "缺少扫描字段"),
    (",".join(FIELDS) + "\n", "为空"),
])
def test_analysis_rejects_missing_fields_and_empty_csv(tmp_path, contents, message):
    path = tmp_path / "invalid.csv"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        analyze_sweep_csv(path)


def test_analysis_cli_writes_json(tmp_path):
    source = tmp_path / "sweep.csv"
    output = tmp_path / "reports" / "analysis.json"
    write_sweep_csv(source, [make_row("one", 0, 0.7, 0)])
    script = Path(__file__).with_name("scenario_analysis.py")

    result = subprocess.run(
        [sys.executable, str(script), str(source), "--output", str(output)],
        capture_output=True, text=True, encoding="utf-8",
    )

    assert result.returncode == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["scenario_count"] == 1
    assert report["factor_summaries"]["initial_speed_m_s"][0]["mean_min_clearance_m"] == pytest.approx(0.7)
