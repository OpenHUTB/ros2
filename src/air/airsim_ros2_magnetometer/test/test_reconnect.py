"""Magnetometer read failure and reconnect state tests."""

from __future__ import annotations

import time
from types import SimpleNamespace
from unittest import mock


def _sample():
    return SimpleNamespace(
        magnetic_field_body=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=3.0),
        magnetic_field_covariance=1e-6,
    )


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
            node._on_timer()
        assert node._client is client
        assert node._read_failures == 4
        node._on_timer()
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
            node._on_timer()
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
        time.sleep(0.1)
        assert node._ensure_connected() is True
        assert node._client is recovered
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
