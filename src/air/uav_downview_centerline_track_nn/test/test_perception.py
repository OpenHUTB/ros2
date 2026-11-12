"""下视感知单元测试。"""

import numpy as np

from uav_downview_centerline_track_nn.perception import DownviewCamera
from uav_downview_centerline_track_nn import track


def test_on_line_offset_near_zero():
    cam = DownviewCamera()
    x = 2.0
    state = type("S", (), {"x": x, "y": track.reference_y(x),
                           "yaw": track.reference_tangent(x)})()
    obs = cam.observe(state)
    assert not obs["lost"]
    assert abs(obs["offset"]) < 0.05


def test_offset_sign():
    """机体在标线右侧时，标线应出现在机体左侧，偏移为正。"""
    cam = DownviewCamera()
    x = 2.0
    state = type("S", (), {"x": x, "y": track.reference_y(x) - 0.5, "yaw": 0.0})()
    obs = cam.observe(state)
    assert obs["offset"] > 0.0


def test_normalize_bounds():
    feat = DownviewCamera.normalize(9.0, 9.0, 9.0)
    assert feat.shape == (3,)
    assert np.all(np.abs(feat) <= 1.0)
