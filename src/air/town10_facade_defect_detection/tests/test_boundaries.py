"""验证会改变检测解释的边界：类别抑制、补齐字节和坐标缩放。"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from town10_facade_defect_detection.detector import FacadeDetector, class_nms, letterbox
from town10_facade_defect_detection.image_transport import decode_image


def test_nms_keeps_different_classes_on_same_region():
    assert class_nms([[0, 0, 10, 10]] * 3, [0.9, 0.8, 0.7], [0, 3, 0]) == [0, 1]


def test_rgb_with_padding_uses_step():
    msg = SimpleNamespace(height=2, width=1, step=5, encoding="rgb8",
                          data=bytes([255, 0, 0, 99, 99, 0, 255, 0, 88, 88]))
    assert decode_image(msg).tolist() == [[[0, 0, 255]], [[0, 255, 0]]]


def test_bad_payload_is_rejected():
    with pytest.raises(ValueError, match="数据长度"):
        decode_image(SimpleNamespace(height=2, width=2, step=6, encoding="bgr8", data=b"short"))


def test_letterbox_non_square_image():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    padded, ratio, x, y = letterbox(image)
    assert (padded.shape, ratio, x, y) == ((640, 640, 3), 1, 0, 140)
    assert padded[0, 0].tolist() == [114, 114, 114]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -0.1, 1.1])
def test_invalid_confidence(value):
    with pytest.raises(ValueError, match="confidence"):
        FacadeDetector(confidence=value)
