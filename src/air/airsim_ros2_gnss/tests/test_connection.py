"""Verify AirSim connection attempts do not block the ROS executor."""

from __future__ import annotations

import sys
import time
from types import SimpleNamespace

import pytest


def test_blocking_connection_runs_in_background(monkeypatch) -> None:
    """A slow confirmConnection must not block the caller."""

    class FakeClient:
        instances = []

        def __init__(self, ip, port, timeout_value):
            self.ip = ip
            self.port = port
            self.timeout_value = timeout_value
            self.__class__.instances.append(self)

        def confirmConnection(self):
            time.sleep(0.15)

    import rclpy
    from airsim_ros2_gnss.main import AirSimGnssBridge

    monkeypatch.setitem(
        sys.modules,
        "airsim",
        SimpleNamespace(MultirotorClient=FakeClient),
    )
    rclpy.init()
    node = None
    try:
        node = AirSimGnssBridge()
        node.connection_timeout = 0.5
        node.reconnect_period = 0.1

        started = time.monotonic()
        assert node._ensure_connected() is False
        assert time.monotonic() - started < 0.1

        time.sleep(0.25)
        assert node._ensure_connected() is True
        assert FakeClient.instances[-1].timeout_value == pytest.approx(0.5)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
