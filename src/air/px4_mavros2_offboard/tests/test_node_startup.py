"""ROS 2 launch smoke test for the MAVROS2 offboard node."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time

import pytest


NODE_NAME = "/px4_mavros2_offboard"
REQUIRED_SERVICES = {
    "/uav/offboard/start",
    "/uav/offboard/abort",
}


def _ros2(*arguments: str, timeout: float = 5.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["ros2", *arguments],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _wait_for_node(timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        result = _ros2("node", "list")
        last_output = result.stdout + result.stderr
        if NODE_NAME in result.stdout.splitlines():
            return
        time.sleep(0.5)
    pytest.fail(f"Node {NODE_NAME} was not discovered; ros2 node list said:\n{last_output}")


def _wait_for_services(timeout: float = 10.0) -> set[str]:
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        result = _ros2("service", "list")
        last_output = result.stdout + result.stderr
        services = set(result.stdout.splitlines())
        if REQUIRED_SERVICES.issubset(services):
            return services
        time.sleep(0.5)
    pytest.fail(
        "Offboard services were not discovered; ros2 service list said:\n"
        f"{last_output}"
    )


def test_launch_starts_node_without_mavros() -> None:
    """Launch without MAVROS and verify the node remains reachable over ROS 2."""
    if shutil.which("ros2") is None:
        pytest.skip("ros2 CLI is not available; source the ROS 2 Humble environment")

    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "px4_mavros2_offboard",
            "offboard.launch.py",
            "start_mavros:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    try:
        _wait_for_node()
        assert process.poll() is None, (
            "offboard launch exited unexpectedly:\n"
            f"{process.stdout.read() if process.stdout else ''}"
        )
        services = _wait_for_services()
        assert REQUIRED_SERVICES.issubset(services)
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
