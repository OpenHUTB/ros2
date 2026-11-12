"""Tests for barometer connection and reconnect state handling."""

from __future__ import annotations

import sys
import time
from types import SimpleNamespace
from unittest import mock

import pytest


def _sample():
    return SimpleNamespace(
        altitude=12.5,
        pressure=101250.0,
        qnh=101325.0,
    )


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


def _make_node(factory=None):
    import rclpy
    from airsim_ros2_barometer.main import AirSimBarometerBridge

    rclpy.init()
    return rclpy, AirSimBarometerBridge(client_factory=factory)


def test_four_failures_keep_client_and_next_sample_recovers() -> None:
    client = FailingClient(failures=4)
    rclpy, node = _make_node()
    try:
        node._client = client
        with mock.patch.object(node, "_publish_sample") as publish:
            for _ in range(4):
                node._on_timer()
            assert node._client is client
            assert node._read_failures == 4
            publish.assert_not_called()

            node._on_timer()
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
            node._on_timer()
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
        time.sleep(0.1)
        assert node._ensure_connected() is True
        assert node._client is recovered
        assert node._read_failures == 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_blocking_connection_does_not_block_and_uses_timeout(monkeypatch) -> None:
    class BlockingClient:
        instances = []

        def __init__(self, ip, port, timeout_value):
            self.ip = ip
            self.port = port
            self.timeout_value = timeout_value
            self.__class__.instances.append(self)

        def confirmConnection(self):
            time.sleep(0.15)

    monkeypatch.setitem(
        sys.modules,
        "airsim",
        SimpleNamespace(MultirotorClient=BlockingClient),
    )
    rclpy, node = _make_node()
    try:
        node.connection_timeout = 0.5
        started = time.monotonic()
        assert node._ensure_connected() is False
        assert time.monotonic() - started < 0.1

        time.sleep(0.25)
        assert node._ensure_connected() is True
        assert BlockingClient.instances[-1].timeout_value == pytest.approx(0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
