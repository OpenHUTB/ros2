"""命令行错误提示、边界实验与神经控制评测回归。"""
import json
from pathlib import Path
import subprocess
import sys

from evaluate import evaluate


def test_speed_error_is_readable():
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("main.py")), "--headless", "--speed", "4"],
        capture_output=True, text=True, encoding="utf-8",
    )
    assert result.returncode == 2
    assert "Traceback" not in result.stderr
    assert "0～3" in result.stderr


def test_contact_experiment_exports_results(tmp_path):
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("main.py")), "--headless",
         "--controller", "mlp", "--speed", "3", "--output", str(tmp_path)],
        capture_output=True, text=True, encoding="utf-8",
    )
    assert result.returncode == 0
    report = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert report["collision_steps"] > 0


def test_neural_evaluation_includes_extrapolation(tmp_path):
    reports = evaluate(tmp_path, "mlp")
    assert len(reports) == 3
    assert all(r["controller"] == "mlp" and r["steps"] == 1000 for r in reports)
    assert reports[1]["collision_steps"] == 0
    assert reports[2]["collision_steps"] > 0
    assert (tmp_path / "mlp_metrics.json").is_file()
    assert (tmp_path / "mlp_braking_curves.png").is_file()
    assert not (tmp_path / "mlp").exists()
