"""下视图像渲染与多帧历史缓存测试。"""

import numpy as np

from uav_e2e_multiframe_image_policy import camera


def test_render_patch_shape_and_range():
    frame = camera.render_patch(1.0, 0.0, 0.0)
    assert frame.shape == (camera.IMAGE_H, camera.IMAGE_W)
    assert frame.min() >= 0.0 and frame.max() <= 1.0


def test_history_pads_with_first_frame():
    hist = camera.FrameHistory()
    frame = camera.render_patch(1.0, 0.0, 0.0)
    hist.update(frame)
    stack = hist.stack()
    assert stack.shape == (camera.FRAME_STACK, camera.IMAGE_H, camera.IMAGE_W)
    assert np.allclose(stack[0], stack[1]) and np.allclose(stack[1], stack[2])


def test_history_keeps_last_frames():
    hist = camera.FrameHistory()
    for k in range(4):
        hist.update(np.full((camera.IMAGE_H, camera.IMAGE_W), k, dtype=float))
    stack = hist.stack()
    assert stack.shape[0] == camera.FRAME_STACK
    assert abs(stack[0, 0, 0] - 1.0) < 1e-9
    assert abs(stack[-1, 0, 0] - 3.0) < 1e-9
