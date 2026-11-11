import math

import pytest

from teleop_control.teleop_logic import TeleopCommandState, ZERO_VELOCITY, decode_key


def test_decode_key_distinguishes_normalized_input_from_eof():
    assert decode_key(b"W") == "w"
    assert decode_key(b"") is None


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("w", (0.5, 0.0)),
        ("S", (-0.5, 0.0)),
        ("a", (0.0, 0.5)),
        ("d", (0.0, -0.5)),
    ],
)
def test_key_mappings_and_case_handling(key, expected):
    state = TeleopCommandState()
    assert state.handle_key(key, now=10.0) == expected


def test_unknown_key_and_space_stop_immediately():
    state = TeleopCommandState()
    state.handle_key("w", now=1.0)
    assert state.handle_key(" ", now=1.1) == ZERO_VELOCITY
    assert state.expire(now=4.0) is None

    state.handle_key("a", now=5.0)
    assert state.handle_key("x", now=5.1) == ZERO_VELOCITY


def test_active_command_expires_once_after_timeout():
    state = TeleopCommandState(command_timeout_sec=0.5)
    state.handle_key("w", now=2.0)

    assert state.expire(now=2.49) is None
    assert state.expire(now=2.5) == ZERO_VELOCITY
    assert state.expire(now=9.0) is None


def test_repeated_key_refreshes_watchdog_deadline():
    state = TeleopCommandState(command_timeout_sec=0.5)
    state.handle_key("w", now=1.0)
    state.handle_key("w", now=1.4)

    assert state.expire(now=1.89) is None
    assert state.expire(now=1.9) == ZERO_VELOCITY


def test_watchdog_stops_at_deadline_when_subtraction_would_round_down():
    state = TeleopCommandState(command_timeout_sec=0.5)
    state.handle_key("w", now=0.2)

    assert state.expire(now=0.7) == ZERO_VELOCITY


@pytest.mark.parametrize("value", [0.0, -1.0, math.inf, -math.inf, math.nan])
def test_motion_and_timeout_parameters_must_be_positive_and_finite(value):
    with pytest.raises(ValueError):
        TeleopCommandState(linear_speed=value)
    with pytest.raises(ValueError):
        TeleopCommandState(angular_speed=value)
    with pytest.raises(ValueError):
        TeleopCommandState(command_timeout_sec=value)


@pytest.mark.parametrize("timestamp", [math.inf, -math.inf, math.nan])
def test_non_finite_timestamps_are_rejected(timestamp):
    state = TeleopCommandState()
    with pytest.raises(ValueError):
        state.handle_key("w", timestamp)
    with pytest.raises(ValueError):
        state.expire(timestamp)


def test_stop_clears_command_and_watchdog_state():
    state = TeleopCommandState()
    state.handle_key("s", now=3.0)

    assert state.stop() == ZERO_VELOCITY
    assert state.velocity == ZERO_VELOCITY
    assert state.expire(now=8.0) is None
