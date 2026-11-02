import numpy as np

from uav_gate_center_servo.core.geometry import camera_label, project_label
from uav_gate_center_servo.core.servo import GateServo, ServoConfig


Q0 = np.array([1.0, 0.0, 0.0, 0.0])


def test_label_forward_sign():
    label = camera_label(np.array([5.0, 0.0, 2.0]), np.array([0.0, 0.0, 2.0]), Q0)
    assert 4.8 < label[0] < 5.0


def test_label_left_up_signs():
    left = camera_label(np.array([5.0, 1.0, 2.0]), np.array([0.0, 0.0, 2.0]), Q0)
    up = camera_label(np.array([5.0, 0.0, 4.0]), np.array([0.0, 0.0, 2.0]), Q0)
    assert left[1] > 0.9
    assert up[2] > 1.9


def test_camera_offset_removed():
    label = camera_label(np.array([1.075, 0.0, 0.012]), np.zeros(3), Q0)
    assert abs(label[0] - 1.0) < 1e-6


def test_servo_signs_and_clamps():
    cmd = GateServo(ServoConfig()).command(np.array([4.0, 1.0, -1.0]))
    assert cmd[0] > 0 and cmd[1] > 0 and cmd[2] < 0
    assert abs(cmd[1]) <= 1.8 and abs(cmd[2]) <= 0.9


def test_servo_invalid_slowdown():
    s = GateServo(ServoConfig())
    assert np.allclose(s.command(np.array([4.0, 1.0, 0.0]), valid_frac=0.0), 0.0)
    assert np.allclose(s.command(np.array([4.0, 1.0, 0.0]), valid_prediction=False), 0.0)


def test_projection_center():
    u, v = project_label(np.array([3.0, 0.0, 0.0]), (48, 64))
    assert abs(u - 31.5) < 1e-6 and abs(v - 23.5) < 1e-6
