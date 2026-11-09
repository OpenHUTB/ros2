"""Sequence-isolated waypoint learning / 按完整序列隔离的航点学习。"""

import json
import math

import pytest
import torch
import numpy as np

from ros2_uav_controller import TrackingCore, load_checkpoint
from rooftop_pv.models import TinyWaypointPlanner
from rooftop_pv.planning import planning_features, route_waypoint_target
from rooftop_pv.training import train_planning


def manifest(path, test_cross_track=0.4):
    rows = []
    for split, sequence, cross in (("train", "train_sequence", 0.2),
                                   ("test", "held_out_sequence", test_cross_track)):
        for index, heading in enumerate((-0.2, 0.0, 0.2)):
            rows.append({"split": split, "sequence_id": sequence, "frame_index": index,
                         "expert_input": {"cross_track_error": cross, "heading_error": heading,
                                          "distance_to_turn": index + 0.5}})
    (path / "manifest.jsonl").write_text("\n".join(map(json.dumps, rows)) + "\n", encoding="utf-8")
    return rows


def test_route_teacher_outputs_body_waypoint_offsets_not_commands():
    assert route_waypoint_target(1.0, 0.0, 10.0) == (2.0, -1.0)
    assert route_waypoint_target(1.0, math.pi / 2, 2.0) == pytest.approx((-1.0, -2.0))
    assert route_waypoint_target(1.0, 0.0, 0.0) == (0.0, -1.0)
    assert planning_features(0.1, 2 * math.pi, 20.0) == pytest.approx((0.1, 0.0, 2.0))


def test_planning_checkpoint_reloads_and_measures_held_out_waypoints(tmp_path):
    manifest(tmp_path)
    result = train_planning(tmp_path, epochs=100, seed=7)
    assert result["train_sequences"] == ["train_sequence"]
    assert result["test_sequences"] == ["held_out_sequence"]
    assert result["output"] == ["body_forward_offset_m", "body_left_offset_m"]
    assert result["waypoint_error_m"] < 0.6
    assert math.isfinite(result["mse"])
    checkpoint = tmp_path / result["checkpoint"]
    model = load_checkpoint(checkpoint, TinyWaypointPlanner)
    with torch.inference_mode():
        prediction = model(torch.tensor([planning_features(0.4, 0.0, 1.5)]))
    assert prediction.shape == (1, 2)
    assert torch.isfinite(prediction).all()
    payload = torch.load(checkpoint, map_location="cpu", weights_only=True)
    assert payload["target"] == "route_expert_2m_lookahead_waypoint"
    core = TrackingCore(planning_model=model)
    assert core.command(0.0)[:2] == (0.0, 0.0)
    core.set_pose(0.0, 0.4, 0.0, "map", 0.0)
    core.set_route([(0.0, 0.0), (5.0, 0.0)], "map")
    core.set_image(np.zeros((4, 4, 3), dtype=np.uint8), 0.0)
    forward, turn, status = core.command(0.0)
    assert 0 < forward <= core.max_speed
    assert -core.max_yaw_rate <= turn < 0
    assert status["planner"] == "trained_local_waypoint"
    assert status["planned_local_waypoint_m"][1] < 0


def test_test_observations_never_affect_fitted_weights(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    first.mkdir()
    second.mkdir()
    manifest(first, test_cross_track=0.4)
    manifest(second, test_cross_track=5.0)
    train_planning(first, epochs=3, seed=42)
    train_planning(second, epochs=3, seed=42)
    left = torch.load(first / "planning.pt", weights_only=True)["state_dict"]
    right = torch.load(second / "planning.pt", weights_only=True)["state_dict"]
    assert left.keys() == right.keys()
    assert all(torch.equal(left[key], right[key]) for key in left)


def test_sequence_leakage_is_rejected_before_training(tmp_path):
    rows = manifest(tmp_path)
    rows[-1]["sequence_id"] = "train_sequence"
    (tmp_path / "manifest.jsonl").write_text("\n".join(map(json.dumps, rows)) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="sequences overlap"):
        train_planning(tmp_path, epochs=1)
    assert not (tmp_path / "planning.pt").exists()
