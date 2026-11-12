"""视觉暗区分析的单元测试，使用合成图像，不依赖 Gazebo."""

import numpy as np

from auv_end2end.avoid_logic import (
    analyze,
    dark_ratios,
    otsu_split,
    split_roi,
)


def make_frame(left_value, right_value, size=(100, 200)):
    """构造左右两半灰度不同的测试图."""
    height, width = size
    gray = np.full((height, width), right_value, dtype=np.uint8)
    gray[:, :width // 2] = left_value
    return gray


def test_split_roi_crops_middle_band():
    """ROI 只保留画面 15% 到 70% 的条带."""
    gray = np.zeros((100, 200), dtype=np.uint8)
    roi = split_roi(gray)
    assert roi.shape[0] == 55
    assert roi.shape[1] == 200


def test_bright_frame_has_no_obstacle():
    """全亮画面不应被判为障碍."""
    gray = np.full((100, 200), 120, dtype=np.uint8)
    left, right, _ = analyze(gray, adaptive=True)
    assert left < 0.01
    assert right < 0.01


def test_all_dark_frame_is_fully_blocked():
    """全黑画面应占满两侧."""
    gray = np.zeros((100, 200), dtype=np.uint8)
    left, right, _ = analyze(gray, adaptive=True)
    assert left > 0.99
    assert right > 0.99


def test_left_half_dark_dominates():
    """左半暗时应只有左侧占比高."""
    gray = make_frame(25, 120)
    left, right, _ = analyze(gray, adaptive=True)
    assert left > 0.9
    assert right < 0.1


def test_fixed_threshold_fails_under_fog():
    """固定阈值 22 在整体灰度被抬高后失效，用于说明自适应阈值的必要性."""
    gray = make_frame(45, 120)
    left, right = dark_ratios(gray, 22)
    assert left == 0.0
    assert right == 0.0


def test_adaptive_threshold_survives_fog():
    """自适应阈值仍能分离出被雾效抬高的障碍."""
    gray = make_frame(45, 120)
    left, right, thresh = analyze(gray, adaptive=True)
    assert left > 0.9
    assert thresh > 22


def test_manual_threshold_is_used_when_adaptive_disabled():
    """关闭自适应时应严格使用给定阈值."""
    gray = make_frame(25, 120)
    left, _, thresh = analyze(gray, fixed_thresh=22, adaptive=False)
    assert thresh == 22
    assert left == 0.0


def test_uniform_water_falls_back_to_fixed():
    """
    开阔水域整幅画面灰度相同（实测 37），Otsu 无解.

    此时必须退回固定阈值，否则整幅水色都会被当成障碍，
    导致 AUV 一直处于避让状态、越跑越偏.
    """
    gray = np.full((100, 200), 37, dtype=np.uint8)
    left, right, thresh = analyze(gray, fixed_thresh=22, adaptive=True)
    assert thresh == 22
    assert left == 0.0
    assert right == 0.0


def test_smooth_fog_gradient_falls_back_to_fixed():
    """只有平滑灰度渐变（雾）而没有障碍时，不应启动自适应分割."""
    row = np.linspace(30, 50, 200).astype(np.uint8)
    gray = np.tile(row, (100, 1))
    _, _, thresh = analyze(gray, fixed_thresh=22, adaptive=True)
    assert thresh == 22


def test_bimodal_frame_uses_otsu_threshold():
    """真有墙体时双峰可分性接近 1，才允许使用 Otsu 阈值."""
    gray = make_frame(45, 120)
    _, _, thresh = analyze(gray, fixed_thresh=22, adaptive=True)
    assert thresh == 45


def test_separability_is_bounded():
    """可分性必须按概率化类间方差计算，不能超过 1."""
    gray = make_frame(45, 120)
    thresh, separability = otsu_split(gray)
    assert thresh == 45
    assert 0.99 <= separability <= 1.0
