"""端到端策略网络测试。"""

import numpy as np

from uav_e2e_multiframe_image_policy import camera, scene
from uav_e2e_multiframe_image_policy.policy import E2EImagePolicy


def _stack(x: float, y: float, yaw: float = 0.0) -> np.ndarray:
    frame = camera.render_patch(x, y, yaw)
    return np.stack([frame, frame, frame])


def test_output_command_ranges():
    net = E2EImagePolicy()
    decision = net.forward(_stack(1.0, scene.centerline_y(1.0), 0.0))
    assert 0.0 <= decision["forward"] <= 1.0
    assert -1.2 <= decision["yaw_cmd"] <= 1.2
    assert len(decision["features"]) == 4


def test_right_offset_produces_left_correction():
    net = E2EImagePolicy()
    decision = net.forward(_stack(1.0, scene.centerline_y(1.0) + 0.4, 0.0))
    assert decision["yaw_cmd"] < 0.0


def test_features_respond_to_offset():
    net = E2EImagePolicy()
    near = net._features(_stack(1.0, scene.centerline_y(1.0), 0.0))
    off = net._features(_stack(1.0, scene.centerline_y(1.0) + 0.4, 0.0))
    assert not np.allclose(near, off)
