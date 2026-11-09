"""多场景扫描测试。Tests for multi-scenario sweeps."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from scenario_sweep import run_sweep


def test_sweep_covers_requested_scenario_combinations():
    report = run_sweep(start_xs=(-8.0, -7.0), wall_xs=(0.0,), speeds=(0.0, 1.5), heights=(1.0,))

    assert report["controller"] == "range"
    assert report["scenario_count"] == 4
    assert report["collision_scenarios"] == 0
    assert len(report["scenarios"]) == 4
    assert {(row["start_x_m"], row["initial_speed_m_s"]) for row in report["scenarios"]} == {
        (-8.0, 0.0), (-8.0, 1.5), (-7.0, 0.0), (-7.0, 1.5),
    }
    assert all(row["controller"] == "range" for row in report["scenarios"])


def test_sweep_rejects_empty_inputs():
    with pytest.raises(ValueError, match="至少需要一个场景组合"):
        run_sweep(start_xs=())


def test_sweep_cli_writes_json(tmp_path):
    output = tmp_path / "nested" / "sweep.json"
    script = Path(__file__).with_name("scenario_sweep.py")
    result = subprocess.run(
        [sys.executable, str(script), "--start-x", "-7", "--speed", "0", "--output", str(output)],
        capture_output=True, text=True, encoding="utf-8",
    )

    assert result.returncode == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["scenario_count"] == 1
    assert payload["scenarios"][0]["start_x_m"] == -7.0
