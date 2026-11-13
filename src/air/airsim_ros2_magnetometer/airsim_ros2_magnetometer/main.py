"""AirSim magnetometer bridge for ROS 2 Humble."""

from __future__ import annotations

import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import MagneticField

from .magnetometer import build_message, parse_magnetometer_sample


class AirSimMagnetometerBridge(Node):
    def __init__(self, client_factory: Optional[Callable[[], object]] = None) -> None:
        super().__init__("airsim_magnetometer_bridge")
        self._client_factory = client_factory
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 41451).value)
        self.vehicle_name = str(self.declare_parameter("vehicle_name", "").value or "")
        self.magnetometer_name = str(
            self.declare_parameter("magnetometer_name", "").value or ""
        )
        self.topic = str(
            self.declare_parameter("magnetometer_topic", "/uav/magnetometer").value
        )
        self.frame_id = str(
            self.declare_parameter("frame_id", "base_link").value or "base_link"
        )
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        self.rpc_timeout = float(
            self.declare_parameter("rpc_timeout", 1.0).value
        )
        self.field_scale = float(self.declare_parameter("field_scale", 1.0).value)
        for name, value in (
            ("publish_rate", self.publish_rate),
            ("reconnect_period", self.reconnect_period),
            ("rpc_timeout", self.rpc_timeout),
            ("field_scale", self.field_scale),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.publish_rate <= 0.0 or self.reconnect_period <= 0.0:
            raise ValueError("rate parameters must be positive")
        if self.rpc_timeout <= 0.0 or self.field_scale <= 0.0:
            raise ValueError("rpc timeout and field scale must be positive")

        self._client = None
        self._read_failures = 0
        self._next_connect_time = 0.0
        self._sample_logged = False
        self._shutting_down = False
        self._rpc_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="airsim-mag-rpc",
        )
        self._connection_future: Future | None = None
        self._read_future: Future | None = None
        self._publisher = self.create_publisher(
            MagneticField,
            self.topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f"AirSim magnetometer bridge started for {self.host}:{self.port}"
        )

    def _ensure_connected(self) -> bool:
        if self._client is not None:
            return True
        if self._connection_future is not None:
            if not self._connection_future.done():
                return False
            connection_future = self._connection_future
            self._connection_future = None
            try:
                client = connection_future.result()
            except ImportError:
                self.get_logger().error(
                    "AirSim Python client is not installed; run "
                    "python3 -m pip install -r requirements.txt",
                    throttle_duration_sec=5.0,
                )
                return False
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot connect to AirSim: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False
            self._client = client
            self._read_failures = 0
            return True
        now = time.monotonic()
        if now < self._next_connect_time:
            return False
        self._next_connect_time = now + self.reconnect_period
        self._connection_future = self._rpc_executor.submit(self._connect_once)
        return False

    def _connect_once(self):
        if self._client_factory is not None:
            client = self._client_factory()
        else:
            import airsim

            client = airsim.MultirotorClient(
                ip=self.host,
                port=self.port,
                timeout_value=self.rpc_timeout,
            )
        client.confirmConnection()
        return client

    def _on_timer(self) -> None:
        if self._shutting_down:
            return
        if not self._ensure_connected() or self._client is None:
            return
        self._complete_read()
        if self._client is None:
            return
        self._schedule_read()

    def _schedule_read(self) -> None:
        """Schedule one AirSim read without blocking the ROS executor."""
        if self._client is None or self._read_future is not None:
            return
        self._read_future = self._rpc_executor.submit(
            self._read_once,
            self._client,
        )

    def _read_once(self, client):
        return client.getMagnetometerData(
            magnetometer_name=self.magnetometer_name,
            vehicle_name=self.vehicle_name,
        )

    def _complete_read(self) -> None:
        read_future = self._read_future
        if read_future is None or not read_future.done():
            return
        self._read_future = None
        try:
            sample = read_future.result()
            normalized = parse_magnetometer_sample(sample)
            if not self._sample_logged:
                self._sample_logged = True
                self.get_logger().debug(
                    f"First magnetometer sample: {normalized.field}"
                )
            self._publisher.publish(
                build_message(
                    normalized,
                    frame_id=self.frame_id,
                    stamp=self.get_clock().now().to_msg(),
                    field_scale=self.field_scale,
                )
            )
            self._read_failures = 0
        except Exception as exc:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._client = None
                self._read_failures = 0
                self._next_connect_time = time.monotonic() + self.reconnect_period
                self.get_logger().warning(f"AirSim magnetometer connection lost: {exc}")
            else:
                self.get_logger().warning(
                    f"AirSim magnetometer read failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def destroy_node(self) -> None:
        self._shutting_down = True
        self._rpc_executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AirSimMagnetometerBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
