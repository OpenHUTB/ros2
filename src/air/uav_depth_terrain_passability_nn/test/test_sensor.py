"""地形深度传感器单元测试。"""

import numpy as np

from uav_depth_terrain_passability_nn.depth_sensor import DepthTerrainSensor


def test_feature_dimension():
    sensor = DepthTerrainSensor()
    state = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    percept = sensor.observe(state)
    assert percept["features"].shape == (9,)
    assert set(percept["sector_points"].keys()) == {"left", "center", "right"}


def test_open_ground_center_low():
    sensor = DepthTerrainSensor()
    state = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    percept = sensor.observe(state)
    center = percept["features"].reshape(3, 3)[1]
    assert center[0] < 0.1
    assert center[1] < 0.1


def test_normalize_bounds():
    raw = np.array([[1.0, 1.0, 1.0]] * 3)
    out = DepthTerrainSensor.normalize(raw.reshape(-1))
    assert np.all(out >= -1.0)
    assert np.all(out <= 1.0)
