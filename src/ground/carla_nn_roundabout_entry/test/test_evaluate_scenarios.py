from __future__ import annotations

import io
import json
from contextlib import redirect_stdout

import pytest

from carla_nn_roundabout_entry.evaluate_scenarios import (
    evaluate_scenarios,
    main,
    phase_for_seed,
    run_episode,
)


def test_controllers_use_the_same_phase_for_each_seed() -> None:
    report = evaluate_scenarios([7, 3], steps=20, controller="both")
    rows = report["scenarios"]
    assert [row["seed"] for row in rows] == [7, 3, 7, 3]
    assert rows[0]["npc_phase_rad"] == rows[2]["npc_phase_rad"] == round(phase_for_seed(7), 6)
    assert rows[1]["npc_phase_rad"] == rows[3]["npc_phase_rad"] == round(phase_for_seed(3), 6)
    assert [row["controller"] for row in rows] == ["rule", "rule", "image-model", "image-model"]


def test_episode_is_repeatable_for_same_seed_and_controller() -> None:
    first = run_episode(11, 50, "rule")
    second = run_episode(11, 50, "rule")
    assert first == second


def test_summary_is_aggregated_from_scenario_rows() -> None:
    report = evaluate_scenarios([3, 7, 11], steps=80, controller="rule")
    rows = report["scenarios"]
    summary = report["summary"]["rule"]
    assert summary["scenario_count"] == 3
    assert summary["collision_rate"] == round(sum(row["collision"] for row in rows) / 3, 3)
    assert summary["finish_rate"] == round(sum(row["finished"] for row in rows) / 3, 3)
    assert summary["mean_steps"] == round(sum(row["steps"] for row in rows) / 3, 3)
    assert summary["worst_minimum_npc_center_distance_m"] == min(
        row["minimum_npc_center_distance_m"] for row in rows
    )


def test_invalid_inputs_are_rejected() -> None:
    with pytest.raises(ValueError, match="seed"):
        evaluate_scenarios([], steps=10)
    with pytest.raises(ValueError, match="steps"):
        evaluate_scenarios([7], steps=0)
    with pytest.raises(ValueError, match="controller"):
        evaluate_scenarios([7], steps=10, controller="gap-policy")


def test_cli_accepts_seeds_steps_and_controller() -> None:
    output = io.StringIO()
    with redirect_stdout(output):
        main(["--seeds", "7", "3", "--steps", "12", "--controller", "rule"])
    report = json.loads(output.getvalue())
    assert report["seeds"] == [7, 3]
    assert report["step_limit"] == 12
    assert list(report["summary"]) == ["rule"]
