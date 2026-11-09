import json
from pathlib import Path

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

