import csv
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from mujoco_motor_temperature_predictor.forecast import fit_and_evaluate, load_samples
from sample_data import write_sample


def write_series(path):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "speed_mps", "left_temperature_c", "right_temperature_c", "left_current_a", "right_current_a", "ambient_c"))
        for index in range(500):
            t = index * 0.1
            temp = 25 + 0.03 * t + 0.6 * np.sin(t / 4)
            current = 1.5 + 0.3 * np.sin(t / 2)
            writer.writerow((t, 0.4 + 0.1 * np.cos(t), temp, temp + 0.1, current, current + 0.1, 24.0))


def test_time_split_forecast_is_repeatable(tmp_path):
    path = tmp_path / "series.csv"
    write_series(path)
    _, features, target = load_samples(path, horizon_s=2.0)
    assert features.shape[1] == 4
    assert len(target) > 400
    result = fit_and_evaluate(path, horizon_s=2.0)
    assert result["train_samples"] >= 20
    assert result["test_samples"] >= 10
    assert np.isfinite(result["rmse_c"])
    package_root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(package_root)
    subprocess.run([
        sys.executable,
        str(package_root / "main.py"),
        "--csv", str(path),
        "--horizon", "2",
        "--model", str(tmp_path / "model.json"),
        "--results", str(tmp_path / "results.json"),
    ], check=True, env=env, capture_output=True, text=True)
    assert (tmp_path / "model.json").is_file()
    assert (tmp_path / "results.json").is_file()


def test_unsorted_time_series_is_rejected(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("time_s,speed_mps,left_temperature_c,left_current_a,ambient_c\n0,0,20,1,20\n1,0,21,1,20\n0.5,0,21,1,20\n", encoding="utf-8")
    with pytest.raises(ValueError, match="strictly increasing"):
        load_samples(path)


def test_sample_generator_creates_cli_compatible_csv(tmp_path):
    path = tmp_path / "sample.csv"
    assert write_sample(path, samples=100, seed=3) == 100
    times, features, targets = load_samples(path, horizon_s=0.5)
    assert len(times) == len(features) == len(targets)


def test_default_cli_predicts_five_seconds_ahead(tmp_path):
    source = tmp_path / "sample.csv"
    write_sample(source, samples=500)
    model = tmp_path / "model.json"
    results = tmp_path / "results.json"
    package_root = Path(__file__).resolve().parents[1]
    process = subprocess.run([
        sys.executable, str(package_root / "main.py"),
        "--csv", str(source), "--model", str(model), "--results", str(results),
    ], check=True, capture_output=True, text=True)
    payload = json.loads(results.read_text(encoding="utf-8"))
    assert payload["horizon_s"] == 5.0
    assert json.loads(model.read_text(encoding="utf-8"))["horizon_s"] == 5.0
    assert "horizon=5.0 s" in process.stdout
    with source.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = np.asarray([float(row["time_s"]) for row in rows])
    expected = [
        float(rows[np.searchsorted(times, time + 5.0)]["left_temperature_c"])
        for time in payload["times_s"]
    ]
    np.testing.assert_allclose(payload["actual_c"], expected)
