"""Regression tests for the ROS-independent UAV message projection."""

import pytest

from ros_messages import (
    TOPIC_BRAKING_ACCEL,
    TOPIC_FRONT_RANGE,
    TOPIC_STATE,
    build_message_payload,
)


def _row():
    return {
        "time_s": 1.25,
        "sensor_range_m": 3.5,
        "closing_speed_m_s": -0.75,
        "height_m": 1.2,
        "accel_m_s2": -1.4,
        "sensor_valid": 1,
    }


def test_topic_names_and_state_field_order_are_stable():
    payload = build_message_payload(_row())

    assert TOPIC_FRONT_RANGE == "uav/front_range_m"
    assert TOPIC_BRAKING_ACCEL == "uav/braking_accel_m_s2"
    assert TOPIC_STATE == "uav/state"
    assert payload.state == (1.25, 3.5, -0.75, 1.2)


def test_scalar_topics_match_state_values():
    payload = build_message_payload(_row())

    assert payload.front_range_m == payload.state[1]
    assert payload.braking_accel_m_s2 == -1.4


@pytest.mark.parametrize("invalid_flag", [0.5, -1, float("nan"), float("inf")])
def test_payload_rejects_non_binary_sensor_flags(invalid_flag):
    row = _row()
    row["sensor_valid"] = invalid_flag

    with pytest.raises(ValueError, match="sensor_valid 必须为 0 或 1"):
        build_message_payload(row)


def test_payload_preserves_control_command_when_sensor_is_invalid():
    row = _row()
    row["sensor_valid"] = 0
    row["accel_m_s2"] = -2.5

    payload = build_message_payload(row)

    assert payload.braking_accel_m_s2 == row["accel_m_s2"]
