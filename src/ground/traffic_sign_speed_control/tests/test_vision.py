"""限速牌定位与字符切分的离线视觉回归测试。

直接调用纯 OpenCV 模块，不启动 ROS，也不加载 PyTorch 权重。
"""
import random

import numpy as np
import pytest
from PIL import Image

from traffic_sign_speed_control.sign_render import augment, draw_limit_sign
from traffic_sign_speed_control.vision import find_sign_roi, split_chars

WIDTH, HEIGHT = 320, 240
CENTER = (WIDTH // 2, HEIGHT // 2)


def _scene(value_kmh, seed=0, **degrade):
    sign_size = 96
    sign = draw_limit_sign(value_kmh, size=sign_size, alpha=True)
    if degrade:
        sign = augment(sign, random.Random(seed), **degrade)
    image = Image.new("RGB", (WIDTH, HEIGHT), (72, 118, 76))
    left = CENTER[0] - sign_size // 2
    top = CENTER[1] - sign_size // 2
    image.paste(sign, (left, top), sign)
    return np.asarray(image)[:, :, ::-1].copy()


@pytest.mark.parametrize("value_kmh", [15, 30, 80])
def test_clean_sign_locates_roi_and_splits_digits(value_kmh):
    bgr = _scene(value_kmh)
    roi = find_sign_roi(bgr)
    assert roi is not None
    cx, cy, radius = roi
    assert abs(cx - CENTER[0]) <= 2
    assert abs(cy - CENTER[1]) <= 2
    assert 43 <= radius <= 49
    assert len(split_chars(bgr, cx, cy, radius)) == len(str(value_kmh))


def test_moderately_degraded_sign_remains_detectable():
    bgr = _scene(
        30,
        seed=7,
        rotate=5,
        blur_sigma=0.8,
        brightness=0.2,
        contrast=0.2,
        noise_std=0.02,
    )
    roi = find_sign_roi(bgr)
    assert roi is not None
    assert len(split_chars(bgr, *roi)) == 2


def test_blank_scene_returns_no_sign():
    blank = np.full((HEIGHT, WIDTH, 3), 100, dtype=np.uint8)
    assert find_sign_roi(blank) is None
    assert split_chars(blank, 1000, 1000, 20) == []
