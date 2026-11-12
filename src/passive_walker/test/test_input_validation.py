"""Input contract tests that do not require MuJoCo or ROS 2."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
from passive_walker.sim import run_headless, validate_finite, validate_positive


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), -float("inf")])
def test_positive_inputs_reject_nonpositive_and_nonfinite(value):
    with pytest.raises(ValueError):
        validate_positive("seconds", value)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_finite_inputs_reject_nonfinite_values(value):
    with pytest.raises(ValueError):
        validate_finite("push_speed", value)


@pytest.mark.parametrize("seconds", [0, -0.1, float("nan"), float("inf")])
def test_run_headless_rejects_invalid_duration_before_loading_mujoco(seconds):
    with pytest.raises(ValueError, match="seconds"):
        run_headless(seconds=seconds)


@pytest.mark.parametrize("argv", [
    ["run", "--seconds", "0"],
    ["run", "--seconds", "nan"],
    ["run", "--push", "inf"],
    ["node", "--period", "-1"],
    ["node", "--period", "nan"],
])
def test_cli_reports_invalid_numeric_arguments(argv):
    with pytest.raises(SystemExit) as exc:
        main.build_parser().parse_args(argv)
    assert exc.value.code == 2


def test_cli_accepts_negative_finite_push_speed():
    args = main.build_parser().parse_args(["run", "--push", "-0.2"])
    assert args.push == -0.2

