"""Tests for barometer connection, reading and message publication."""

from __future__ import annotations

import sys
import threading
import time
from types import SimpleNamespace
from unittest import mock

import pytest


def _sample():
    return SimpleNamespace(
        altitude=12.5,
        pressure=101250.0,
        qnh=1013.25,
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

    def getBarometerData(self, **_kwargs):
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

    def getBarometerData(self, **_kwargs):
        self.read_entered.set()
        if not self.release_read.wait(timeout=1.0):
            raise RuntimeError("test did not release the read")
        return _sample()


def _make_node(factory=None):
    import rclpy
    from airsim_ros2_barometer.main import AirSimBarometerBridge

    rclpy.init()
    return rclpy, AirSimBarometerBridge(client_factory=factory)


def test_invalid_port_is_rejected() -> None:
    import rclpy
    from airsim_ros2_barometer.main import AirSimBarometerBridge

    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    try:
        with pytest.raises(ValueError, match="port"):
            AirSimBarometerBridge()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_four_failures_keep_client_and_next_sample_recovers() -> None:
    client = FailingClient(failures=4)
    rclpy, node = _make_node()
    try:
        node._client = client
        with mock.patch.object(node, "_publish_sample") as publish:
            for _ in range(4):
                node._schedule_read()
                _wait_for_future(node._read_future)
                node._complete_read()
            assert node._client is client
            assert node._read_failures == 4
            publish.assert_not_called()

            node._schedule_read()
            _wait_for_future(node._read_future)
            node._complete_read()
            assert node._client is client
            assert node._read_failures == 0
            publish.assert_called_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_fifth_failure_disconnects_and_waits_for_reconnect() -> None:
    client = FailingClient(failures=100)
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


def test_reconnect_restores_client_and_resets_failures() -> None:
    recovered = FailingClient(failures=0)
    rclpy, node = _make_node(factory=lambda: recovered)
    try:
        node._next_connect_time = 0.0
        assert node._ensure_connected() is False
        _wait_for_future(node._connection_future)
        assert node._ensure_connected() is True
        assert node._client is recovered
        assert node._read_failures == 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_blocking_connection_does_not_block_and_uses_timeout(
    monkeypatch,
) -> None:
    class BlockingClient:
        instances = []
        entered = threading.Event()
        release = threading.Event()

        def __init__(self, ip, port, timeout_value):
            self.ip = ip
            self.port = port
            self.timeout_value = timeout_value
            self.__class__.instances.append(self)

        def confirmConnection(self):
            self.__class__.entered.set()
            if not self.__class__.release.wait(timeout=1.0):
                raise RuntimeError("test did not release the connection")

    monkeypatch.setitem(
        sys.modules,
        "airsim",
        SimpleNamespace(MultirotorClient=BlockingClient),
    )
    rclpy, node = _make_node()
    try:
        node.rpc_timeout = 0.5
        started = time.monotonic()
        assert node._ensure_connected() is False
        assert time.monotonic() - started < 0.25
        assert BlockingClient.entered.wait(timeout=1.0)

        BlockingClient.release.set()
        _wait_for_future(node._connection_future)
        assert node._ensure_connected() is True
        assert BlockingClient.instances[-1].timeout_value == pytest.approx(0.5)
    finally:
        BlockingClient.release.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_blocking_read_does_not_block_timer() -> None:
    client = BlockingReadClient()
    rclpy, node = _make_node()
    try:
        node._client = client
        with mock.patch.object(node, "_publish_sample") as publish:
            started = time.monotonic()
            node._on_timer()
            assert time.monotonic() - started < 0.25
            assert client.read_entered.wait(timeout=1.0)

            started = time.monotonic()
            node._on_timer()
            assert time.monotonic() - started < 0.25
            publish.assert_not_called()

            client.release_read.set()
            _wait_for_future(node._read_future)
            node._on_timer()
            publish.assert_called_once()
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
        assert "airsim_ros2_barometer/requirements.txt" in message
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_publish_sample_maps_all_messages() -> None:
    rclpy, node = _make_node()
    try:
        node.pressure_variance = 2.25
        node._pressure_publisher = mock.Mock()
        node._altitude_publisher = mock.Mock()
        node._qnh_publisher = mock.Mock()
        sample = SimpleNamespace(
            altitude="12.5",
            pressure="101250.0",
            qnh="1013.25",
        )

        with mock.patch.object(node.get_logger(), "info") as log_info:
            node._publish_sample(sample)
            node._publish_sample(sample)

        pressure = node._pressure_publisher.publish.call_args.args[0]
        altitude = node._altitude_publisher.publish.call_args.args[0]
        qnh = node._qnh_publisher.publish.call_args.args[0]
        assert pressure.header.frame_id == "base_link"
        assert pressure.fluid_pressure == pytest.approx(101250.0)
        assert pressure.variance == pytest.approx(2.25)
        assert altitude.data == pytest.approx(12.5)
        assert qnh.data == pytest.approx(101325.0)
        log_info.assert_called_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
