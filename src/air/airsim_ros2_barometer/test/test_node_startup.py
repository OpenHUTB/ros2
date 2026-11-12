"""ROS 2 launch smoke test for the AirSim barometer bridge."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time

import pytest


NODE_NAME = "/airsim_barometer_bridge"
REQUIRED_TOPICS = {
    "/uav/barometer/pressure",
    "/uav/barometer/altitude",
    "/uav/barometer/qnh",
}


def _ros2(*arguments: str, timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["ros2", *arguments],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _wait_for_node_and_topics(timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    last_nodes = ""
    last_topics = ""
    while time.monotonic() < deadline:
        nodes_result = _ros2("node", "list")
        topics_result = _ros2("topic", "list")
        last_nodes = nodes_result.stdout + nodes_result.stderr
        last_topics = topics_result.stdout + topics_result.stderr
        node_found = NODE_NAME in nodes_result.stdout.splitlines()
        topics_found = REQUIRED_TOPICS.issubset(
            set(topics_result.stdout.splitlines())
        )
        if node_found and topics_found:
            return
        time.sleep(0.5)
    pytest.fail(
        f"Node/topics not discovered.\nros2 node list:\n{last_nodes}\n"
        f"ros2 topic list:\n{last_topics}"
    )


def test_launch_starts_without_airsim() -> None:
    """Verify the launch entry remains alive without an AirSim server."""
    if shutil.which("ros2") is None:
        pytest.skip("ros2 CLI is not available; source the ROS 2 Humble environment")

    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "airsim_ros2_barometer",
            "bringup.launch.py",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    try:
        _wait_for_node_and_topics()
        assert process.poll() is None, (
            "AirSim barometer launch exited unexpectedly:\n"
            f"{process.stdout.read() if process.stdout else ''}"
        )
    finally:
        if process.poll() is None:
            if os.name == "posix":
                os.killpg(process.pid, signal.SIGINT)
            else:
                process.terminate()
            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                if os.name == "posix":
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                process.wait(timeout=5.0)
