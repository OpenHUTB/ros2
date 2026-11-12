"""车道线感知与判据的单元测试（不需要 CARLA 服务端）。

覆盖：
- `_band_positions`：左右车道线的定位与「同一条线被劈成两半」的过滤
- `_band_center`：两侧齐全时取中点；只有一侧时用历史宽度估计（单侧估计）
- 判据配置：PASS 阈值必须写在 params 里且合理
"""
import os

import numpy as np
import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "carla_lane_following", "app.py")
LANE_TAG = 24


@pytest.fixture(scope="module")
def fn():
    src = open(APP, encoding="utf-8").read()
    start = src.index("def _band_positions(")
    end = src.index("def semantic_lane_offset(")
    ns = {"np": np}
    exec(compile(src[start:end], APP, "exec"), ns)  # noqa: S102
    return ns


def make_tags(height=100, width=200, left_x=50, right_x=150, tag=LANE_TAG):
    tags = np.zeros((height, width), dtype=np.uint8)
    if left_x is not None:
        tags[:, left_x] = tag
    if right_x is not None:
        tags[:, right_x] = tag
    return tags


def test_both_lines_found(fn):
    tags = make_tags()
    xl, xr = fn["_band_positions"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200)
    assert (xl, xr) == (50, 150)


def test_missing_right_line(fn):
    tags = make_tags(right_x=None)
    xl, xr = fn["_band_positions"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200)
    assert xl == 50 and xr is None


def test_double_line_split_is_filtered(fn):
    """两条线离得太近时视为同一条线被劈开，只保留右侧（避免中点算到线内侧）。"""
    tags = make_tags(left_x=95, right_x=100)
    xl, xr = fn["_band_positions"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200)
    assert xl is None and xr == 100


def test_center_when_both_sides(fn):
    tags = make_tags()
    state = {}
    center, src = fn["_band_center"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200, state)
    assert center == pytest.approx(100.0)
    assert src == "dual"
    assert state["width"] == pytest.approx(100.0)


def test_single_side_uses_history_width(fn):
    """只有左侧线时，用上一帧记下的车道宽度补出中心（单侧估计）。"""
    state = {"width": 100.0}
    tags = make_tags(right_x=None)
    center, src = fn["_band_center"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200, state)
    assert src == "left_only"
    assert center == pytest.approx(100.0)      # 50 + 100/2


def test_no_line_returns_none(fn):
    tags = np.zeros((100, 200), dtype=np.uint8)
    center, src = fn["_band_center"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200, {})
    assert center is None and src == "none"


def test_wrong_tag_is_ignored(fn):
    tags = make_tags(tag=1)
    xl, xr = fn["_band_positions"](tags, 0.5, 1.0, LANE_TAG, 10, 100, 200)
    assert xl is None and xr is None


def test_evaluation_thresholds_are_in_params():
    with open(os.path.join(ROOT, "config", "params.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    ev = cfg["evaluation"]
    assert 0 < float(ev["max_mean_offset"]) < 1.0
    assert 0 < float(ev["max_lost_ratio"]) <= 1.0
    assert isinstance(ev["require_complete"], bool)
    # 运行时长与「跑满」判据要自洽
    assert float(cfg["control"]["max_duration"]) > 0
