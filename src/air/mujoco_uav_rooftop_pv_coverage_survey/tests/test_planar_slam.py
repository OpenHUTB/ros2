"""SLAM estimation and keyboard event tests / SLAM估计与键盘事件测试。"""

import numpy as np
import pytest

from rooftop_pv.keyboard import KeyboardState
from rooftop_pv.slam import PlanarLandmarkSLAM, detect_landmarks, run_slam_navigation


def test_landmark_reobservation_corrects_biased_odometry_without_true_pose():
    slam = PlanarLandmarkSLAM(observation_std=0.01)
    slam.observe(7, np.array([2.0, 1.0]))
    slam.predict(np.array([1.2, 0.0]), motion_std=0.2)
    # A new camera measurement, not a true-pose update, constrains the movement.
    slam.observe(7, np.array([1.0, 1.0]))
    assert abs(slam.position[0] - 1.0) < 0.002
    assert abs(slam.position[1]) < 1e-8
    assert np.allclose(slam.landmark_map()[7], [2, 1], atol=0.002)
    assert np.linalg.eigvalsh(slam.covariance).min() >= -1e-12
    assert slam.reobservations == 1


def test_depth_backprojection_is_metric_and_does_not_require_world_pose():
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    rgb[48:52, 68:72] = [255, 0, 0]
    depth = np.ones((100, 100)) * 5
    measurements = detect_landmarks(rgb, depth, 90.0)
    assert np.allclose(measurements[0], [2, 0], atol=1e-8)


def test_keyboard_events_turn_move_and_expire(monkeypatch):
    monkeypatch.setattr("rooftop_pv.keyboard.time.monotonic", lambda: 5.0)
    keyboard = KeyboardState(timeout=0.4)
    keyboard.on_key(ord("W"))
    assert keyboard.command(5.2)["forward_speed"] > 0
    assert keyboard.command(5.5) == {"forward_speed": 0, "yaw_rate": 0}
    keyboard.on_key(ord("A"))
    assert keyboard.command(5.2)["yaw_rate"] > 0
    keyboard.on_key(32)
    assert keyboard.command(5.2) == {"forward_speed": 0, "yaw_rate": 0}


def test_slam_rejects_nonfinite_measurements():
    slam = PlanarLandmarkSLAM()
    with pytest.raises(ValueError):
        slam.observe(1, np.array([float("nan"), 0]))


def test_rendered_rgbd_slam_closes_navigation_loop(tmp_path):
    result = run_slam_navigation(tmp_path)
    assert result["navigation_completed"]
    assert result["landmark_reobservations"] > 0
    assert result["position_rmse_m"] < result["odometry_rmse_m"]
    assert result["final_position_error_m"] < 0.15
    assert (tmp_path / "slam_landmarks.json").is_file()
    assert (tmp_path / "slam_navigation.gif").is_file()
