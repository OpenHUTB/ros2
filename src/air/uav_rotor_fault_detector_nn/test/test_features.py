"""特征与滑窗的单元测试（不依赖 MuJoCo / torch）。"""

import numpy as np
import pytest

from uav_rotor_fault_detector_nn.features import (
    FEATURE_DIM,
    FEATURE_SCALE,
    GRAVITY,
    ROTOR_COUNT,
    build_step_features,
    make_windows,
    quat_to_matrix,
    specific_force,
)


def test_quat_to_matrix_identity():
    np.testing.assert_allclose(quat_to_matrix([1.0, 0.0, 0.0, 0.0]), np.eye(3), atol=1e-12)


def test_quat_to_matrix_yaw_90():
    half = np.pi / 4.0
    quat = [np.cos(half), 0.0, 0.0, np.sin(half)]
    rot = quat_to_matrix(quat)
    # 绕 z 轴转 90°：机体系 x 轴指向世界系 y 轴
    np.testing.assert_allclose(rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


def test_quat_to_matrix_zero_is_identity():
    np.testing.assert_allclose(quat_to_matrix([0.0, 0.0, 0.0, 0.0]), np.eye(3))


def test_specific_force_at_rest_measures_gravity():
    quat = [1.0, 0.0, 0.0, 0.0]
    vel = np.array([0.0, 0.0, 1.5])
    acc = specific_force(vel, vel, 0.01, quat)
    np.testing.assert_allclose(acc, [0.0, 0.0, GRAVITY], atol=1e-9)


def test_specific_force_level_flight_zero_vertical_accel():
    quat = [1.0, 0.0, 0.0, 0.0]
    prev = np.array([0.0, 0.0, 0.0])
    cur = np.array([1.0, 0.0, 0.0])
    acc = specific_force(cur, prev, 0.1, quat)
    np.testing.assert_allclose(acc, [10.0, 0.0, GRAVITY], atol=1e-9)


def test_specific_force_rotated_frame():
    quat = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
    acc = specific_force([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.01, quat)
    # 静止时无论姿态如何，机体系读数都应是 R^T(g z) = g * (第三行)
    np.testing.assert_allclose(acc, quat_to_matrix(quat).T @ np.array([0.0, 0.0, GRAVITY]))


def test_build_step_features_shape_and_scaling():
    max_thrust = 6.0
    features = build_step_features(
        acc_body=[1.2, 0.0, -0.5],
        gyro=[0.1, -0.2, 0.3],
        vel=[0.5, -0.4, 0.1],
        cmd_thrusts=[6.0, 3.0, 0.0, 1.5],
        max_thrust=max_thrust,
    )
    assert features.shape == (FEATURE_DIM,)
    assert np.all(np.isfinite(features))
    # 前 9 维按尺度归一化
    np.testing.assert_allclose(features[:9], np.array([1.2, 0.0, -0.5, 0.1, -0.2, 0.3, 0.5, -0.4, 0.1])
                               / np.array(FEATURE_SCALE[:9]))
    # 后 4 维是推力指令 / 最大推力
    np.testing.assert_allclose(features[9:], [1.0, 0.5, 0.0, 0.25])


def test_build_step_features_clips_command():
    features = build_step_features([0, 0, 0], [0, 0, 0], [0, 0, 0], [99.0] * ROTOR_COUNT, 6.0)
    assert np.all(features[9:] <= 1.5)


def test_make_windows_shapes_and_labels():
    n_steps = 250
    features = np.arange(n_steps * FEATURE_DIM, dtype=np.float32).reshape(n_steps, FEATURE_DIM)
    labels = np.tile(np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32), (n_steps, 1))
    labels[120:, 1] = 0.0

    windows, targets, ends = make_windows(features, labels, window=100, stride=10)

    assert windows.shape == (len(ends), 100, FEATURE_DIM)
    assert targets.shape == (len(ends), ROTOR_COUNT)
    assert ends[0] == 99 and ends[-1] == 249
    # 标签取窗口最后一帧
    np.testing.assert_allclose(targets, labels[ends])
    # 窗口内容确实是 [end-99, end]
    np.testing.assert_allclose(windows[0], features[0:100])


def test_make_windows_too_short_returns_empty():
    features = np.zeros((10, FEATURE_DIM), dtype=np.float32)
    labels = np.ones((10, ROTOR_COUNT), dtype=np.float32)
    windows, targets, ends = make_windows(features, labels, window=100, stride=10)
    assert windows.shape == (0, 100, FEATURE_DIM)
    assert targets.shape == (0, ROTOR_COUNT)
    assert ends.shape == (0,)


@pytest.mark.parametrize("window,stride", [(0, 10), (100, 0), (-1, 5)])
def test_make_windows_rejects_bad_params(window, stride):
    features = np.zeros((250, FEATURE_DIM), dtype=np.float32)
    labels = np.ones((250, ROTOR_COUNT), dtype=np.float32)
    with pytest.raises(ValueError):
        make_windows(features, labels, window=window, stride=stride)
