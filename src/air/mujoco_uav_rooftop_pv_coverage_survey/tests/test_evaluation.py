from pathlib import Path

from rooftop_pv.dataset import DatasetConfig, generate_dataset
from rooftop_pv.evaluation import evaluate_scenarios
from rooftop_pv.route import RouteConfig


def test_scenario_evaluation_has_all_layouts(tmp_path: Path):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3))
    result = evaluate_scenarios(tmp_path)
    assert set(result) == {"dense", "occluded", "sparse"}
    assert all(item["frames"] == 3 for item in result.values())


def test_cli_evaluate_writes_metrics(tmp_path: Path):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3))
    from main import main
    assert main(["--mode", "evaluate", "--output", str(tmp_path)]) == 0
    assert (tmp_path / "evaluation_metrics.json").is_file()
