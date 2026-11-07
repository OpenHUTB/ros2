from __future__ import annotations

import pytest

from evaluate_scenarios import evaluate_scenarios
from main import RoundaboutWorld, run_headless_with_controller


def test_evaluation_preserves_seed_order_and_matches_simulator_metrics() -> None:
    report = evaluate_scenarios([7, 3], steps=20)

    assert report["scenario_count"] == 2
    assert report["controller"] == "gap-policy"
    assert [row["seed"] for row in report["results"]] == [7, 3]
    assert report["results"][0] == {"seed": 7, **run_headless_with_controller(7, 20)}
    assert report["results"][1] == {"seed": 3, **run_headless_with_controller(3, 20)}


def test_policy_controller_records_a_decision_for_each_step() -> None:
    report = evaluate_scenarios([7], steps=80)
    result = report["results"][0]

    assert result["controller_policy_frames"] == 80
    assert result["policy_wait_frames"] + result["policy_enter_frames"] == 80




def test_evaluation_requires_seeds() -> None:
    with pytest.raises(ValueError, match="随机种子"):
        evaluate_scenarios([])


def test_evaluation_requires_positive_step_count() -> None:
    with pytest.raises(ValueError, match="仿真步数"):
        evaluate_scenarios([7], steps=0)
