"""节点行为测试：参数解析、话题规范化、保存与异常处理（用桩，无需 ROS）。"""

from __future__ import annotations

import pytest

import conftest
import signal

from conftest import IMSHOW_CALLS, StubNode, WRITTEN
from image_recorder import main as rec


@pytest.fixture(autouse=True)
def reset_state():
    StubNode.PARAMS = {}
    WRITTEN.clear()
    IMSHOW_CALLS.clear()
    conftest.IMWRITE_RESULT[0] = True
    yield
    StubNode.PARAMS = {}


def _params(**kwargs):
    StubNode.PARAMS = kwargs


def test_defaults_and_topic():
    node = rec.ImageRecorder()
    assert node.image_topic == "/carla/ego_vehicle/camera/rgb/front/image"
    assert node.show_window is True
    assert node.image_format == "jpg"


def test_topic_without_slash_is_normalized():
    _params(image_topic="camera/front")
    node = rec.ImageRecorder()
    assert node.image_topic == "/camera/front"
    assert node.subscription_topic == "/camera/front"


def test_headless_mode_saves_every_frame(tmp_path):
    _params(show_window=False, output_dir=str(tmp_path), image_format="png")
    node = rec.ImageRecorder()
    node.listener_callback(object())
    node.listener_callback(object())
    assert len(WRITTEN) == 2
    assert IMSHOW_CALLS == []
    assert all(p.endswith(".png") for p in WRITTEN)
    assert WRITTEN[0] != WRITTEN[1]
    assert (tmp_path).is_dir()


def test_window_mode_uses_imshow(tmp_path):
    _params(show_window=True, output_dir=str(tmp_path))
    node = rec.ImageRecorder()
    node.listener_callback(object())
    assert IMSHOW_CALLS == ["Image Recorder"]
    assert len(WRITTEN) == 1


def test_save_index_increments(tmp_path):
    _params(show_window=False, output_dir=str(tmp_path))
    node = rec.ImageRecorder()
    node.listener_callback(object())
    node.listener_callback(object())
    assert node._save_index == 2


def test_spin_external_shutdown_is_clean(tmp_path, monkeypatch):
    """Humble 的 spin 会抛 ExternalShutdownException，main 应干净退出。"""
    _params(show_window=False, output_dir=str(tmp_path))
    StubNode.INSTANCES.clear()

    def boom(_node):
        raise rec.ExternalShutdownException()

    monkeypatch.setattr(rec.rclpy, 'spin', boom)
    previous = signal.getsignal(signal.SIGINT)
    try:
        rec.main([])                      # 不应抛异常
        # main 在清理阶段把 SIGINT 置为忽略；断言后用 finally 还原，
        # 避免污染 pytest 进程自己的信号处理。
        assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN
        assert StubNode.INSTANCES[-1].destroyed is True
    finally:
        signal.signal(signal.SIGINT, previous)


def test_write_failure_is_reported(tmp_path):
    _params(show_window=False, output_dir=str(tmp_path))
    conftest.IMWRITE_RESULT[0] = False
    node = rec.ImageRecorder()
    node.listener_callback(object())
    assert len(WRITTEN) == 1
    assert node._save_index == 0              # 失败不推进序号
    assert any('Failed to write' in e for e in node.get_logger().errors)


def test_callback_reports_bridge_error(tmp_path):
    _params(show_window=False, output_dir=str(tmp_path))
    node = rec.ImageRecorder()

    class BrokenBridge:
        def imgmsg_to_cv2(self, msg, encoding):
            raise RuntimeError("bad image")

    node.bridge = BrokenBridge()
    node.listener_callback(object())
    assert len(WRITTEN) == 0
    assert any("bad image" in e for e in node.get_logger().errors)
