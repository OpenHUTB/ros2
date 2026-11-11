import math

import numpy as np

from quad_cascade_pid_hover_wind.geometry import (euler_to_quat, quat_to_euler,
                                                  quat_to_rotation,
                                                  rotation_to_quat, wrap_angle)


def test_quat_euler_roundtrip():
    for roll, pitch, yaw in [(0.1, -0.2, 0.3), (0.0, 0.0, 1.5), (-0.4, 0.1, -1.0)]:
        q = euler_to_quat(roll, pitch, yaw)
        r, p, y = quat_to_euler(q)
        assert abs(r - roll) < 1e-6
        assert abs(p - pitch) < 1e-6
        assert abs(y - yaw) < 1e-6


def test_rotation_orthonormal():
    q = euler_to_quat(0.3, -0.2, 1.0)
    R = quat_to_rotation(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9


def test_rotation_to_quat_roundtrip():
    q = euler_to_quat(0.2, 0.3, -0.5)
    R = quat_to_rotation(q)
    q2 = rotation_to_quat(R)
    r1, p1, y1 = quat_to_euler(q)
    r2, p2, y2 = quat_to_euler(q2)
    assert abs(r1 - r2) < 1e-6
    assert abs(p1 - p2) < 1e-6
    assert abs(y1 - y2) < 1e-6


def test_wrap_angle():
    assert abs(wrap_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-9
    assert abs(wrap_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-9
    assert wrap_angle(0.0) == 0.0
