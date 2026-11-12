import csv

import pytest

from motor_thermal_scenario_evaluator.evaluate import evaluate
from motor_thermal_scenario_evaluator.plot import plot_curves


def test_evaluation_writes_independent_scenario_metrics_and_plot(tmp_path):
    summary = evaluate(steps=6000, output=tmp_path)
    assert len(summary) == 6
    with (tmp_path / "scenario_curves.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 6 * 6000
    assert {row["mode"] for row in rows} == {"fixed_speed", "derated"}
    hot = [row for row in summary if row["scenario"] == "hot_high_load"]
    assert hot[1]["derated_percent"] > 0
    assert hot[1]["peak_temperature_c"] < hot[0]["peak_temperature_c"]
    image = tmp_path / "curves.png"
    plot_curves(tmp_path / "scenario_curves.csv", image)
    assert image.stat().st_size > 10_000


def test_invalid_scenario_duration_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="steps"):
        evaluate(steps=0, output=tmp_path)
