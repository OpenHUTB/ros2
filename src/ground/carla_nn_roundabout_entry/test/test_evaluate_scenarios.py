from __future__ import annotations

import pytest

from evaluate_scenarios import evaluate_scenarios
from main import run_headless


def test_evaluation_preserves_seed_order_and_matches_simulator_metrics() -> None:
    report = evaluate_scenarios([7, 3], steps=20)

    assert report["scenario_count"] == 2
    assert [row["seed"] for row in report["results"]] == [7, 3]
    assert report["results"][0] == {"seed": 7, **run_headless(7, 20)}
    assert report["results"][1] == {"seed": 3, **run_headless(3, 20)}


def test_evaluation_requires_seeds() -> None:
    with pytest.raises(ValueError, match="随机种子"):
        evaluate_scenarios([])


def test_evaluation_requires_positive_step_count() -> None:
    with pytest.raises(ValueError, match="仿真步数"):
        evaluate_scenarios([7], steps=0)
