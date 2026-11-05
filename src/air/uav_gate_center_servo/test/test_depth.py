import numpy as np

from uav_gate_center_servo.core.depth import (depth_to_u8, downsample_depth,
                                              valid_fraction)


def test_downsample_48x64_shape():
    d = np.full((48, 64), 3.0, np.float32)
    q = downsample_depth(d)
    assert q.shape == (12, 16)
    assert q.dtype == np.float32


def test_downsample_640x480_shape():
    d = np.full((480, 640), 4.0, np.float32)
    q = downsample_depth(d)
    assert q.shape == (12, 16)
    assert abs(float(q.mean()) - 4.0) < 1e-6


def test_invalid_pixels_ignored():
    d = np.zeros((2, 2), np.float32)
    d[0, 0] = 5.0
    q = downsample_depth(d, (1, 1))
    assert abs(float(q[0, 0]) - 5.0) < 1e-6


def test_valid_fraction_and_u8():
    d = np.array([[0.0, 2.0], [np.nan, 3.0]], np.float32)
    assert abs(valid_fraction(d) - 0.5) < 1e-9
    q = depth_to_u8(d)
    assert q[0, 0] == 0 and q[0, 1] > 0
