import cv2
import numpy as np

from mujoco_uav_moving_pad_landing.landing import (
    controller_command,
    detect_colored_center,
    expert_command,
    vision_features,
)


def test_expert_command_is_bounded_and_descends_when_aligned():
    features = np.array([0.0, 0.0, 0.7, 0.0, 0.0], dtype=np.float32)
    command = expert_command(features)
    assert np.all(np.abs(command[:2]) <= 0.55)
    assert command[2] < 0.0


def test_missing_visual_markers_produce_finite_features():
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    features = vision_features(image, altitude=1.0, pad_velocity=(0.1, -0.1))
    assert features.shape == (6,)
    assert np.isfinite(features).all()
    np.testing.assert_array_equal(controller_command(features), np.zeros(3, dtype=np.float32))


def test_invalid_visual_marker_skips_trained_model():
    class ModelThatMustNotRun:
        def predict(self, _features):
            raise AssertionError("invalid vision must not reach the trained model")

    features = np.array([0.3, -0.2, 0.5, 0.1, 0.1, 0.0], dtype=np.float32)
    np.testing.assert_array_equal(controller_command(features, ModelThatMustNotRun()), np.zeros(3, dtype=np.float32))


def test_nonfinite_features_produce_zero_velocity():
    features = np.array([0.3, np.nan, 0.5, 0.1, 0.1, 1.0], dtype=np.float32)
    np.testing.assert_array_equal(controller_command(features), np.zeros(3, dtype=np.float32))


def test_hsv_detector_finds_pad_and_uav_centers():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 20), (39, 49), (0, 255, 0), -1)
    cv2.rectangle(image, (60, 60), (79, 79), (255, 0, 0), -1)
    pad = detect_colored_center(image, "pad")
    uav = detect_colored_center(image, "uav")
    assert pad is not None and uav is not None
    assert abs(pad[0] + 0.255) < 0.02
    assert abs(uav[0] - 0.2) < 0.02
