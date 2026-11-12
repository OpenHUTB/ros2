"""ROS-independent payload projection for the UAV ranging node."""

from dataclasses import dataclass
from typing import Mapping, Tuple


TOPIC_FRONT_RANGE = "uav/front_range_m"
TOPIC_BRAKING_ACCEL = "uav/braking_accel_m_s2"
TOPIC_STATE = "uav/state"


@dataclass(frozen=True)
class UavMessagePayload:
    """Values published by the ROS 2 node for one simulation step."""

    front_range_m: float
    braking_accel_m_s2: float
    state: Tuple[float, float, float, float]


def build_message_payload(row: Mapping[str, object]) -> UavMessagePayload:
    """Project one simulation row into the documented ROS message values."""

    sensor_valid = float(row["sensor_valid"])
    if sensor_valid not in (0.0, 1.0):
        raise ValueError("sensor_valid 必须为 0 或 1")

    return UavMessagePayload(
        front_range_m=float(row["sensor_range_m"]),
        braking_accel_m_s2=float(row["accel_m_s2"]),
        state=(
            float(row["time_s"]),
            float(row["sensor_range_m"]),
            float(row["closing_speed_m_s"]),
            float(row["height_m"]),
        ),
    )
