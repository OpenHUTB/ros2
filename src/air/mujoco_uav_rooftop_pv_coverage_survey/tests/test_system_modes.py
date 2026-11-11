import json
from pathlib import Path

import yaml

from rooftop_pv.dataset import DatasetConfig, generate_dataset
from rooftop_pv.keyboard import command_for_key
from rooftop_pv.mapping import map_and_navigate
from rooftop_pv.route import RouteConfig


def test_keyboard_mapping_is_safe():
    assert command_for_key("w")["forward_speed"] > 0
    assert command_for_key(" ") == {"forward_speed": 0.0, "yaw_rate": 0.0}
    assert command_for_key("unknown") == {"forward_speed": 0.0, "yaw_rate": 0.0}


def test_sensor_mapping_and_navigation_replay(tmp_path: Path):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3))
    metrics = map_and_navigate(tmp_path)
    assert metrics["sequences"] == 3
    assert metrics["mapped_cells"] > 0
    assert 0.0 < metrics["map_coverage"] <= 1.0
    assert metrics["route_replay_progress"] == 1.0


def test_route_replay_progress_tracks_missing_frames(tmp_path: Path):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3))
    manifest = tmp_path / "manifest.jsonl"
    rows = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines()]
    manifest.write_text(
        "".join(json.dumps(row) + "\n" for row in rows if row["frame_index"] < 2),
        encoding="utf-8",
    )

    metrics = map_and_navigate(tmp_path)

    assert metrics["route_replay_progress"] == 2 / 3


def test_all_mode_writes_complete_summary(tmp_path: Path):
    from main import main, read_config, DEFAULT_CONFIG

    config = read_config(DEFAULT_CONFIG)
    config.update(frames_per_scene=3, image_width=32, image_height=24)
    config_path = tmp_path / "survey.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    output = tmp_path / "results"

    assert main(["--config", str(config_path), "--output", str(output), "--mode", "all"]) == 0
    summary = json.loads((output / "day2_summary.json").read_text(encoding="utf-8"))
    assert set(summary) == {"dataset", "evaluation", "mapping", "segmentation", "planning", "control", "end_to_end", "keyboard", "slam"}
    assert summary["dataset"]["quality_status"] == "pass"
    assert summary["mapping"]["route_replay_progress"] == 1.0
    assert (output / summary["keyboard"]["gif"]).is_file()
    assert summary["slam"]["completed_waypoints"] == summary["slam"]["total_waypoints"]
