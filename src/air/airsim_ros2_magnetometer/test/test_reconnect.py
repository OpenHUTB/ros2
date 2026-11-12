"""Magnetometer read failure and reconnect state tests."""

from __future__ import annotations

import sys
import threading
import time
from types import SimpleNamespace
from unittest import mock


def _sample():
    return SimpleNamespace(
        magnetic_field_body=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=3.0),
        magnetic_field_covariance=1e-6,
    )


def _wait_until(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition was not met before the deadline")


def _wait_for_future(future) -> None:
    assert future is not None
    _wait_until(future.done)


class FailingClient:
    def __init__(self, failures):
        self.failures = failures

    def confirmConnection(self):
        return None

    def getMagnetometerData(self, **_kwargs):
        if self.failures:
            self.failures -= 1
            raise RuntimeError("read failed")
        return _sample()


class BlockingReadClient:
    def __init__(self):
        self.read_entered = threading.Event()
        self.release_read = threading.Event()

    def confirmConnection(self):
        return None

    def getMagnetometerData(self, **_kwargs):
        self.read_entered.set()
        if not self.release_read.wait(timeout=1.0):
            raise RuntimeError("test did not release the read")
        return _sample()


def _make_node(factory=None):
    import rclpy
    from airsim_ros2_magnetometer.main import AirSimMagnetometerBridge

    rclpy.init()
    return rclpy, AirSimMagnetometerBridge(client_factory=factory)


def test_four_failures_recover_without_reconnect() -> None:
    client = FailingClient(4)
    rclpy, node = _make_node()
    try:
        node._client = client
        node._publisher = mock.Mock()
        for _ in range(4):
            node._schedule_read()
            _wait_for_future(node._read_future)
            node._complete_read()
        assert node._client is client
        assert node._read_failures == 4

        node._schedule_read()
        _wait_for_future(node._read_future)
        node._complete_read()
        assert node._read_failures == 0
        node._publisher.publish.assert_called_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_fifth_failure_disconnects() -> None:
    client = FailingClient(100)
    rclpy, node = _make_node()
    try:
        node._client = client
        for _ in range(5):
            node._schedule_read()
            _wait_for_future(node._read_future)
            node._complete_read()
        assert node._client is None
        assert node._read_failures == 0
        assert node._next_connect_time > time.monotonic()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_reconnect_restores_client() -> None:
    recovered = FailingClient(0)
    rclpy, node = _make_node(factory=lambda: recovered)
    try:
        node._next_connect_time = 0.0
        assert node._ensure_connected() is False
        _wait_for_future(node._connection_future)
        assert node._ensure_connected() is True
        assert node._client is recovered
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_blocking_read_does_not_block_timer() -> None:
    client = BlockingReadClient()
    rclpy, node = _make_node()
    try:
        node._client = client
        node._publisher = mock.Mock()
        started = time.monotonic()
        node._on_timer()
        assert time.monotonic() - started < 0.25
        assert client.read_entered.wait(timeout=1.0)

        started = time.monotonic()
        node._on_timer()
        assert time.monotonic() - started < 0.25
        node._publisher.publish.assert_not_called()

        client.release_read.set()
        _wait_for_future(node._read_future)
        node._on_timer()
        node._publisher.publish.assert_called_once()
    finally:
        client.release_read.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_missing_airsim_client_reports_install_hint(monkeypatch) -> None:
    monkeypatch.setitem(sys.modules, "airsim", None)
    rclpy, node = _make_node()
    try:
        node._next_connect_time = 0.0
        with mock.patch.object(node.get_logger(), "error") as log_error:
            assert node._ensure_connected() is False
            _wait_for_future(node._connection_future)
            assert node._ensure_connected() is False

        assert node._client is None
        message = log_error.call_args.args[0]
        assert "not installed" in message
        assert "pip install -r requirements.txt" in message
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
