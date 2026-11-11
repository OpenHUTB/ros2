"""测距几何一致性测试。Tests for range sensor geometry consistency."""

import csv
import json
from pathlib import Path
import subprocess
import sys

import pytest

from range_sensor_quality import analyze_trajectory


FIELDS = ["range_m", "sensor_range_m", "sensor_valid"]


def write_trajectory(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def test_analysis_compensates_mount_offset_and_ignores_invalid_measurements(tmp_path):
    path = tmp_path / "trajectory.csv"
    write_trajectory(path, [
        {"range_m": "2.0", "sensor_range_m": "1.99", "sensor_valid": "1"},
        {"range_m": "1.0", "sensor_range_m": "0.98", "sensor_valid": "1"},
        {"range_m": "0.5", "sensor_range_m": "nan", "sensor_valid": "0"},
    ])

    report = analyze_trajectory(path)

    assert report["samples"] == 3
    assert report["valid_samples"] == 2
    assert report["invalid_samples"] == 1
    assert report["valid_rate"] == pytest.approx(2 / 3)
    assert report["mean_bias_m"] == pytest.approx(-0.005)
    assert report["mae_m"] == pytest.approx(0.005)
    assert report["rmse_m"] == pytest.approx((0.00005) ** 0.5)
    assert report["p95_abs_error_m"] == pytest.approx(0.01)


@pytest.mark.parametrize("rows, message", [
    ([{"range_m": "1", "sensor_range_m": "1", "sensor_valid": "0.5"}], "必须为 0 或 1"),
    ([{"range_m": "1", "sensor_range_m": "inf", "sensor_valid": "1"}], "有限值"),
    ([{"range_m": "1", "sensor_range_m": "nan", "sensor_valid": "0"}], "没有有效测距样本"),
])
def test_analysis_rejects_invalid_flags_values_and_empty_valid_set(tmp_path, rows, message):
    path = tmp_path / "trajectory.csv"
    write_trajectory(path, rows)
    with pytest.raises(ValueError, match=message):
        analyze_trajectory(path)


def test_analysis_rejects_missing_fields_and_empty_csv(tmp_path):
    missing = tmp_path / "missing.csv"
    missing.write_text("range_m,sensor_valid\n1,1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="缺少轨迹字段"):
        analyze_trajectory(missing)

    empty = tmp_path / "empty.csv"
    empty.write_text(",".join(FIELDS) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="为空"):
        analyze_trajectory(empty)


def test_analysis_cli_writes_json(tmp_path):
    source = tmp_path / "trajectory.csv"
    write_trajectory(source, [
        {"range_m": "1", "sensor_range_m": "0.99", "sensor_valid": "1"},
    ])
    output = tmp_path / "reports" / "quality.json"
    script = Path(__file__).with_name("range_sensor_quality.py")

    result = subprocess.run(
        [sys.executable, str(script), str(source), "--output", str(output)],
        capture_output=True, text=True, encoding="utf-8",
    )

    assert result.returncode == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["valid_samples"] == 1
    assert report["mae_m"] == pytest.approx(0.0)
