"""Tests for the ATO energy-work decomposition."""

from __future__ import annotations

import csv
import math
import tempfile
from pathlib import Path

import pytest

from train_ato_stop import scenarios, sim
from train_ato_stop.energy import energy_breakdown
from train_ato_stop.main import _write_energy_report
from train_ato_stop.train import TrainSpec


def test_flat_breakdown_matches_reported_traction_energy():
    spec = TrainSpec()
    result = sim.run(scenarios.flat(), spec=spec)
    breakdown = energy_breakdown(result, spec)
    assert breakdown.traction_work_kwh == pytest.approx(
        result.metrics["energy_kwh"], rel=0.01)
    assert breakdown.resistance_work_kwh > 0.0
    assert breakdown.acceleration_work_kwh > 0.0
    assert breakdown.deceleration_work_kwh > 0.0


def test_grade_scenario_has_negative_net_grade_work():
    result = sim.run(scenarios.grade())
    breakdown = energy_breakdown(result)
    assert breakdown.grade_work_kwh < 0.0


def test_energy_breakdown_is_finite_for_all_scenarios():
    for scenario in scenarios.all_scenarios():
        breakdown = energy_breakdown(sim.run(scenario))
        assert all(math.isfinite(value)
                   for value in breakdown.as_dict().values())
        assert breakdown.endpoint_kinetic_energy_kwh >= 0.0


def test_energy_report_writes_csv_and_markdown():
    spec = TrainSpec()
    results = [sim.run(scenarios.flat(), spec=spec)]
    breakdowns = {results[0].name: energy_breakdown(results[0], spec)}
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = Path(tmp) / "energy.csv"
        md_path = Path(tmp) / "energy.md"
        _write_energy_report(str(csv_path), str(md_path), results, breakdowns)
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 1
        assert rows[0]["scenario"] == "flat"
        assert float(rows[0]["traction_work_kwh"]) > 0.0
        assert "报告牵引" in md_path.read_text(encoding="utf-8")
