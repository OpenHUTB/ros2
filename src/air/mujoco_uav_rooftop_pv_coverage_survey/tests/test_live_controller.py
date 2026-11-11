"""Controller state, model persistence and perception regression tests."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from ros2_uav_controller import TrackingCore, image_rgb, load_checkpoint, main, predict_command, predict_image_command
from rooftop_pv.dataset import DatasetConfig, generate_dataset
from rooftop_pv.models import TinyImageController, TinyMLP, TinyUNet
from rooftop_pv.route import RouteConfig
from rooftop_pv.training import train_control, train_end_to_end, train_segmentation


def test_tracking_stops_for_missing_stale_mismatched_or_finished_input():
    core = TrackingCore()
    assert core.command(0.0)[:2] == (0.0, 0.0)
    core.set_pose(0.0, 1.0, 0.0, "map", 0.0)
    assert core.command(0.0)[2]["state"] == "waiting_route"
    core.set_route([(0.0, 0.0), (10.0, 0.0)], "odom")
    assert core.command(0.0)[2]["state"] == "frame_mismatch"
    core.set_route([(0.0, 0.0), (10.0, 0.0)], "map")
    assert core.command(0.0)[2]["state"] == "waiting_image"
    core.set_image(np.zeros((4, 4, 3), dtype=np.uint8), 0.0)
    forward, turn, status = core.command(0.0)
    assert 0 < forward <= core.max_speed
    assert -core.max_yaw_rate <= turn < 0
    assert status["controller"] == "geometric_tracking_no_checkpoint"
    assert core.command(2.0)[2]["state"] == "stale_pose"
    core.set_pose(10.0, 0.0, 0.0, "map", 2.0)
    assert core.command(2.0)[:2] == (0.0, 0.0)
    assert core.command(2.0)[2]["state"] == "arrived"
    core.set_pose(math.nan, 0.0, 0.0, "map", 2.0)
    assert core.command(2.0)[2]["state"] == "waiting_pose"


def test_bgr_image_with_padding_is_decoded_and_perceived():
    message = SimpleNamespace(encoding="bgr8", width=2, height=1, step=8,
                              data=bytes([76, 40, 10, 20, 20, 20, 0, 0]))
    rgb = image_rgb(message)
    assert rgb.tolist() == [[[10, 40, 76], [20, 20, 20]]]
    core = TrackingCore()
    core.set_image(rgb, 0.0)
    assert core.perception["pv_pixel_ratio"] == 0.5
    assert core.command(2.0)[2]["perception"]["state"] == "stale_image"
    message.step = 5
    with pytest.raises(ValueError):
        image_rgb(message)


def test_unknown_or_untrained_checkpoint_is_rejected(tmp_path: Path):
    path = tmp_path / "random.pt"
    torch.save(TinyMLP().state_dict(), path)
    with pytest.raises(ValueError, match="checkpoint model"):
        load_checkpoint(path, TinyMLP)
    torch.save({"format_version": 1, "model": "TinyMLP", "epochs": 0,
                "state_dict": TinyMLP().state_dict()}, path)
    with pytest.raises(ValueError, match="trained model"):
        load_checkpoint(path, TinyMLP)


def test_training_saves_all_models_and_replay_loads_checkpoint(tmp_path: Path, capsys):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3, image_width=16, image_height=16))
    results = [train_control(tmp_path, epochs=1), train_segmentation(tmp_path, epochs=1),
               train_end_to_end(tmp_path, epochs=1)]
    classes = [TinyMLP, TinyUNet, TinyImageController]
    for result, cls in zip(results, classes):
        checkpoint = tmp_path / result["checkpoint"]
        assert checkpoint.is_file()
        model = load_checkpoint(checkpoint, cls)
        assert not model.training
    model = load_checkpoint(tmp_path / "control.pt", TinyMLP)
    expected = predict_command(model, 0.1, 0.0, 1.0)
    replay = tmp_path / "replay.jsonl"
    replay.write_text(json.dumps({"cross_track_error": 0.1, "heading_error": 0.0, "distance_to_turn": 1.0}) + "\n", encoding="utf-8")
    assert main(["--replay", str(replay), "--checkpoint", str(tmp_path / "control.pt")]) == 0
    prediction = json.loads(capsys.readouterr().out)
    assert prediction["controller"] == "trained_mlp"
    assert prediction["forward_speed"] == expected[0]
    assert prediction["yaw_rate"] == expected[1]
    core = TrackingCore(control_model=model, segmentation_model=load_checkpoint(tmp_path / "segmentation.pt", TinyUNet))
    core.set_pose(0.0, 1.0, 0.0, "map", 0.0)
    core.set_route([(0.0, 0.0), (10.0, 0.0)], "map")
    core.set_image(np.zeros((7, 9, 3), dtype=np.uint8), 0.0)
    forward, turn, status = core.command(0.0)
    assert 0 <= forward <= core.max_speed
    assert abs(turn) <= core.max_yaw_rate
    assert status["controller"] == "trained_mlp"
    assert status["perception"]["source"] == "trained_segmentation"


def test_replay_without_checkpoint_uses_deterministic_expert(tmp_path: Path, capsys):
    replay = tmp_path / "replay.jsonl"
    replay.write_text(json.dumps({"cross_track_error": 0.1, "heading_error": 0.0, "distance_to_turn": 1.0}) + "\n", encoding="utf-8")
    main(["--replay", str(replay)])
    first = capsys.readouterr().out
    main(["--replay", str(replay)])
    assert capsys.readouterr().out == first
    assert json.loads(first)["controller"] == "geometric_expert"


@pytest.mark.parametrize("model_class", [None, TinyMLP, TinyImageController])
def test_all_live_controllers_require_a_fresh_image(model_class):
    kwargs = {}
    if model_class is TinyMLP:
        kwargs["control_model"] = model_class()
    elif model_class is TinyImageController:
        kwargs["end_to_end_model"] = model_class()
    core = TrackingCore(**kwargs)
    core.set_pose(0.0, 0.0, 0.0, "map", 0.0)
    core.set_route([(0.0, 0.0), (10.0, 0.0)], "map")
    assert core.command(0.0)[:2] == (0.0, 0.0)
    assert core.command(0.0)[2]["state"] == "waiting_image"
    core.set_image(np.zeros((4, 4, 3), dtype=np.uint8), 0.0)
    core.set_pose(0.0, 0.0, 0.0, "map", 2.0)
    assert core.command(2.0)[:2] == (0.0, 0.0)
    assert core.command(2.0)[2]["state"] == "stale_image"


def test_live_image_controller_uses_trained_checkpoint(tmp_path):
    generate_dataset(tmp_path, RouteConfig(), DatasetConfig(frames_per_scene=3, image_width=16, image_height=16))
    result = train_end_to_end(tmp_path, epochs=1)
    model = load_checkpoint(tmp_path / result["checkpoint"], TinyImageController)
    rgb = np.full((8, 10, 3), 80, dtype=np.uint8)
    expected = predict_image_command(model, rgb)
    core = TrackingCore(end_to_end_model=model, max_speed=0.2, max_yaw_rate=0.1)
    core.set_pose(0.0, 0.0, 0.0, "map", 0.0)
    core.set_route([(0.0, 0.0), (5.0, 0.0)], "map")
    core.set_image(rgb, 0.0)
    forward, turn, status = core.command(0.0)
    assert forward == pytest.approx(max(0.0, min(0.2, expected[0])))
    assert turn == pytest.approx(max(-0.1, min(0.1, expected[1])))
    assert status["controller"] == "trained_image"
    assert status["state"] == "tracking"
    assert (forward, turn) != (0.2, 0.0)
