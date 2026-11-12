"""Launch and background connection tests for magnetometer bridge."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


def test_launch_starts_without_airsim() -> None:
    if shutil.which("ros2") is None:
        pytest.skip("ros2 CLI is not available")
    process = subprocess.Popen(
        ["ros2", "launch", "airsim_ros2_magnetometer", "bringup.launch.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            nodes = subprocess.run(
                ["ros2", "node", "list"], capture_output=True, text=True
            )
            topics = subprocess.run(
                ["ros2", "topic", "list"], capture_output=True, text=True
            )
            if (
                "/airsim_magnetometer_bridge" in nodes.stdout.splitlines()
                and "/uav/magnetometer" in topics.stdout.splitlines()
            ):
                return
            time.sleep(0.5)
        pytest.fail("magnetometer node/topic was not discovered")
    finally:
        if process.poll() is None:
            if os.name == "posix":
                os.killpg(process.pid, signal.SIGINT)
            else:
                process.terminate()
            process.wait(timeout=10.0)


def test_connection_is_nonblocking(monkeypatch) -> None:
    class BlockingClient:
        instances = []

        def __init__(self, ip, port, timeout_value):
            self.timeout_value = timeout_value
            self.__class__.instances.append(self)

        def confirmConnection(self):
            time.sleep(0.15)

    monkeypatch.setitem(
        sys.modules,
        "airsim",
        SimpleNamespace(MultirotorClient=BlockingClient),
    )
    import rclpy
    from airsim_ros2_magnetometer.main import AirSimMagnetometerBridge

    rclpy.init()
    node = None
    try:
        node = AirSimMagnetometerBridge()
        from rclpy.qos import ReliabilityPolicy

        assert node._publisher.qos_profile.reliability == (
            ReliabilityPolicy.RELIABLE
        )
        node.connection_timeout = 0.5
        started = time.monotonic()
        assert node._ensure_connected() is False
        assert time.monotonic() - started < 0.1
        time.sleep(0.25)
        assert node._ensure_connected() is True
        assert BlockingClient.instances[-1].timeout_value == pytest.approx(0.5)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
