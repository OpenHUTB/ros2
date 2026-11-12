"""参考轨迹与位姿偏差测试。"""

import math

from uav_e2e_multiframe_image_policy import scene


def test_centerline_origin_zero():
    assert abs(scene.centerline_y(0.0)) < 1e-9


def test_lateral_offset_matches_line():
    x = 3.0
    off = scene.lateral_offset(x, scene.centerline_y(x) + 0.3)
    assert abs(off - 0.3) < 1e-9


def test_heading_error_wraps():
    err = scene.heading_error(2.0, 2.0 * math.pi - 0.1)
    assert abs(err) < 0.5
