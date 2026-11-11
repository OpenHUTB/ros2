from pathlib import Path

import pytest
import torch

from navigation import OccupancyGrid
from neural_navigation import LocalPolicy, NeuralPlanner
from slam import RangeHit, SynchronizedSlamNavigation


def test_mlp_policy_emits_straight_for_clear_forward_goal():
    action = NeuralPlanner().predict(OccupancyGrid(width=12, height=12), (6, 6), (10, 6), 0.0)
    assert action.label == "STRAIGHT"


def test_policy_masks_blocked_turns():
    grid = OccupancyGrid(width=12, height=12, occupied={(5, 6), (6, 7)})
    action = NeuralPlanner().predict(grid, (6, 6), (10, 6), 0.0)
    assert action.label == "RIGHT"


def test_planner_rejects_wrong_checkpoint_label_order(tmp_path: Path):
    path = tmp_path / "invalid.pt"
    torch.save({
        "version": 1, "labels": ("STRAIGHT", "LEFT", "RIGHT"),
        "feature_names": NeuralPlanner.feature_names, "hidden_size": 16,
        "state_dict": LocalPolicy().state_dict(),
    }, path)
    with pytest.raises(ValueError, match="label order"):
        NeuralPlanner(path)


def test_policy_checkpoint_emits_valid_controls():
    action = NeuralPlanner().predict(OccupancyGrid(width=12, height=12), (6, 6), (6, 10), 1.5707963267948966)
    assert action.label in NeuralPlanner.labels
    assert action.angular >= 0.0

def test_slam_and_neural_planner_are_updated_in_one_step():
    loop = SynchronizedSlamNavigation(OccupancyGrid(width=12, height=12), (10, 6))
    action = loop.step(0.5, 0.0, 0.0, [RangeHit(1.0, 0.0)])
    assert len(loop.slam.trajectory) == 2
    assert action.label in NeuralPlanner.labels
