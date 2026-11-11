"""验证 ROS 话题载荷与仿真记录的映射。"""

import pytest

from ros_message_data import build_message_data


def test_build_message_data_preserves_topic_values_and_state_order():
    row = {
        "time_s": 1.25,
        "sensor_range_m": 2.4,
        "closing_speed_m_s": 0.75,
        "height_m": 1.2,
        "accel_m_s2": -1.5,
        "sensor_valid": 1,
    }

    messages = build_message_data(row)

    assert messages["front_range_m"] == 2.4
    assert messages["front_range_m"] == messages["state"][1]
    assert messages["braking_accel_m_s2"] == -1.5
    assert messages["state"] == [1.25, 2.4, 0.75, 1.2]
    assert len(messages["state"]) == 4


def test_invalid_sensor_publishes_maximum_reverse_braking():
    row = {
        "time_s": 0.01,
        "sensor_range_m": 20.0,
        "closing_speed_m_s": 0.0,
        "height_m": 1.2,
        "accel_m_s2": 0.0,
        "sensor_valid": 0,
    }

    messages = build_message_data(row)

    assert messages["braking_accel_m_s2"] == -2.5
    assert messages["front_range_m"] == messages["state"][1]


def test_build_message_data_rejects_non_binary_sensor_flag():
    row = {
        "time_s": 0.0,
        "sensor_range_m": 1.0,
        "closing_speed_m_s": 0.0,
        "height_m": 1.2,
        "accel_m_s2": 0.0,
        "sensor_valid": 0.5,
    }

    with pytest.raises(ValueError, match="sensor_valid 必须为 0 或 1"):
        build_message_data(row)
