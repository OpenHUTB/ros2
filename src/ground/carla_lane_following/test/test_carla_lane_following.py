"""车道线偏移计算的单元测试（不需要 CARLA 服务端）。

用合成的语义分割图验证 `_band_center`：左右两条线应在正确位置被识别，
偏移为 0 时中点应落在画面中心。
"""
import os

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, 'carla_lane_following', 'app.py')
LANE_TAG = 24


@pytest.fixture(scope='module')
def band_center():
    src = open(APP, encoding='utf-8').read()
    start = src.index('def _band_center(')
    end = src.index('def semantic_lane_offset(')
    ns = {}
    exec(compile(src[start:end], APP, 'exec'), ns)  # noqa: S102
    return ns['_band_center']


def make_tags(height=100, width=200, left_x=50, right_x=150, tag=LANE_TAG):
    tags = np.zeros((height, width), dtype=np.uint8)
    tags[:, left_x] = tag
    tags[:, right_x] = tag
    return tags


def test_centered_lines_give_image_center(band_center):
    tags = make_tags()
    mid = tags.shape[1] // 2
    center = band_center(tags, 0.5, 1.0, LANE_TAG, min_col=10, mid=mid, wd=tags.shape[1])
    assert center == pytest.approx(mid, abs=1)


def test_shifted_lines_shift_center(band_center):
    tags = make_tags(left_x=20, right_x=120)
    mid = tags.shape[1] // 2
    center = band_center(tags, 0.5, 1.0, LANE_TAG, min_col=10, mid=mid, wd=tags.shape[1])
    assert center < mid          # 两条线整体偏左时，中点也应偏左
    assert center == pytest.approx(70.0, abs=1)


def test_missing_line_returns_none(band_center):
    tags = np.zeros((100, 200), dtype=np.uint8)
    tags[:, 50] = LANE_TAG      # 只有左侧一条线
    mid = 100
    assert band_center(tags, 0.5, 1.0, LANE_TAG, min_col=10, mid=mid, wd=200) is None


def test_wrong_tag_is_ignored(band_center):
    tags = make_tags(tag=1)     # 不是 RoadLines
    assert band_center(tags, 0.5, 1.0, LANE_TAG, min_col=10, mid=100, wd=200) is None
