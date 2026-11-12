"""命名/目录工具测试。"""

from __future__ import annotations

from datetime import datetime

import pytest

from image_recorder.capture import (
    ALLOWED_EXTENSIONS,
    ensure_output_dir,
    make_capture_path,
    normalize_topic,
)


def test_normalize_topic_adds_slash_and_strips():
    assert normalize_topic("carla/camera") == "/carla/camera"
    assert normalize_topic("  /carla/camera  ") == "/carla/camera"


@pytest.mark.parametrize("bad", ["", "   "])
def test_normalize_topic_rejects_empty(bad):
    with pytest.raises(ValueError):
        normalize_topic(bad)


def test_normalize_topic_rejects_non_string():
    with pytest.raises(TypeError):
        normalize_topic(None)


def test_ensure_output_dir_creates_nested(tmp_path):
    target = tmp_path / "a" / "b" / "c"
    result = ensure_output_dir(target)
    assert result == target
    assert target.is_dir()


def test_ensure_output_dir_rejects_file(tmp_path):
    f = tmp_path / "not_a_dir"
    f.write_text("x", encoding="utf-8")
    with pytest.raises(ValueError):
        ensure_output_dir(f)


def test_capture_path_format(tmp_path):
    stamp = datetime(2026, 11, 11, 9, 30, 15, 123456)
    path = make_capture_path(tmp_path, stamp=stamp, extension="png")
    assert path.name == "capture_20261111_093015_123.png"
    assert path.parent == tmp_path
    assert path.exists() is False          # 只生成路径，不写文件


def test_capture_path_with_index(tmp_path):
    stamp = datetime(2026, 1, 2, 3, 4, 5, 6789)
    p0 = make_capture_path(tmp_path, stamp=stamp, index=0)
    p1 = make_capture_path(tmp_path, stamp=stamp, index=1)
    assert p0.name.endswith("_000.jpg")
    assert p1.name.endswith("_001.jpg")
    assert p0 != p1


def test_capture_path_rejects_bad_extension(tmp_path):
    with pytest.raises(ValueError):
        make_capture_path(tmp_path, extension="bmp")


def test_capture_path_accepts_all_allowed(tmp_path):
    for ext in ALLOWED_EXTENSIONS:
        path = make_capture_path(tmp_path, extension=ext)
        assert path.suffix == f".{ext}"


def test_capture_path_rejects_negative_index(tmp_path):
    with pytest.raises(ValueError):
        make_capture_path(tmp_path, index=-1)
