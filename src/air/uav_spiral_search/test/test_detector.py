"""Unit tests for the HSV detector using synthetic images (no ROS required)."""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from uav_spiral_search.target_detector import Settings, detect_target

FRAME_W, FRAME_H = 640, 480


def frame_with_rect(hue_deg, rect=(200, 150, 120, 90),
                    sat=200, val=200, background=40):
    """深灰背景上画一个纯色矩形，返回 (图像, 矩形参数)。"""
    frame = np.full((FRAME_H, FRAME_W, 3), background, dtype=np.uint8)
    x, y, w, h = rect
    hsv_color = np.array([[[hue_deg / 2.0, sat, val]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
    frame[y:y + h, x:x + w] = bgr
    return frame, rect


def test_settings_rejects_invalid_values():
    with pytest.raises(ValueError):
        Settings(hue_min=200)
    with pytest.raises(ValueError):
        Settings(sat_min=300)
    with pytest.raises(ValueError):
        Settings(min_area_fraction=0.9)
    with pytest.raises(ValueError):
        Settings(morph_kernel=0)


def test_detects_default_red_blob():
    frame, (x, y, w, h) = frame_with_rect(356.0)  # 红色 H≈356°
    detection = detect_target(frame)
    assert detection is not None
    expected_cx = 2.0 * (x + w / 2.0) / FRAME_W - 1.0
    expected_cy = 2.0 * (y + h / 2.0) / FRAME_H - 1.0
    assert detection.cx == pytest.approx(expected_cx, abs=0.02)
    assert detection.cy == pytest.approx(expected_cy, abs=0.02)
    assert detection.area_fraction == pytest.approx(
        w * h / (FRAME_W * FRAME_H), rel=0.2)


def test_rejects_empty_scene():
    frame = np.full((FRAME_H, FRAME_W, 3), 40, dtype=np.uint8)
    assert detect_target(frame) is None


def test_rejects_wrong_colour():
    frame, _ = frame_with_rect(120.0)  # 绿色，不在默认红色区间内
    assert detect_target(frame) is None


def test_rejects_blob_below_area_threshold():
    frame, _ = frame_with_rect(356.0, rect=(0, 0, 10, 10))
    assert detect_target(frame) is None


def test_plain_hue_range_detects_orange():
    frame, (x, y, w, h) = frame_with_rect(30.0)  # 橙色 H=30°
    detection = detect_target(frame, Settings(hue_min=10, hue_max=40))
    assert detection is not None
    assert detection.pixel_area > 0


def test_largest_blob_wins():
    frame, big = frame_with_rect(356.0, rect=(300, 200, 160, 120))
    # 在左上角另画一个同色小色块，检测必须选中更大的那个。
    hsv_color = np.array([[[356.0 / 2.0, 200, 200]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
    frame[10:50, 10:50] = bgr
    detection = detect_target(frame)
    assert detection is not None
    expected_cx = 2.0 * (big[0] + big[2] / 2.0) / FRAME_W - 1.0
    assert detection.cx == pytest.approx(expected_cx, abs=0.03)


def test_returns_none_for_malformed_input():
    assert detect_target(None) is None
    assert detect_target(np.zeros((10, 10), dtype=np.uint8)) is None
