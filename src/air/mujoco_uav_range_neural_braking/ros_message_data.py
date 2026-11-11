"""将仿真记录映射为 ROS 话题载荷。Map a simulation row to ROS topic payloads."""


def build_message_data(row):
    """返回测距、制动加速度和状态话题的数据。"""
    sensor_valid = float(row["sensor_valid"])
    if sensor_valid not in (0.0, 1.0):
        raise ValueError("sensor_valid 必须为 0 或 1 / sensor_valid must be 0 or 1")

    sensor_range = float(row["sensor_range_m"])
    return {
        "front_range_m": sensor_range,
        "braking_accel_m_s2": -2.5 if sensor_valid == 0.0 else float(row["accel_m_s2"]),
        "state": [
            float(row["time_s"]),
            sensor_range,
            float(row["closing_speed_m_s"]),
            float(row["height_m"]),
        ],
    }
