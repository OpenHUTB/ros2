"""AirSim barometer bridge for ROS 2 Humble."""

from __future__ import annotations

import math
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

from .barometer import parse_barometer_sample


class AirSimBarometerBridge(Node):
    """Publish AirSim barometer pressure, altitude and QNH."""

    def __init__(self, client_factory: Optional[Callable[[], object]] = None) -> None:
        super().__init__("airsim_barometer_bridge")
        self._client_factory = client_factory
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 41451).value)
        self.vehicle_name = str(
            self.declare_parameter("vehicle_name", "").value or ""
        )
        self.barometer_name = str(
            self.declare_parameter("barometer_name", "").value or ""
        )
        self.pressure_topic = str(
            self.declare_parameter(
                "pressure_topic",
                "/uav/barometer/pressure",
            ).value
            or "/uav/barometer/pressure"
        )
        self.altitude_topic = str(
            self.declare_parameter(
                "altitude_topic",
                "/uav/barometer/altitude",
            ).value
            or "/uav/barometer/altitude"
        )
        self.qnh_topic = str(
            self.declare_parameter(
                "qnh_topic",
                "/uav/barometer/qnh",
            ).value
            or "/uav/barometer/qnh"
        )
        self.frame_id = str(
            self.declare_parameter("frame_id", "base_link").value
            or "base_link"
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 20.0).value
        )
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        self.rpc_timeout = float(
            self.declare_parameter("rpc_timeout", 1.0).value
        )
        self.pressure_variance = float(
            self.declare_parameter("pressure_variance", 1.0).value
        )

        for name, value in (
            ("publish_rate", self.publish_rate),
            ("reconnect_period", self.reconnect_period),
            ("rpc_timeout", self.rpc_timeout),
            ("pressure_variance", self.pressure_variance),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.reconnect_period <= 0.0:
            raise ValueError("reconnect_period must be positive")
        if self.rpc_timeout <= 0.0:
            raise ValueError("rpc_timeout must be positive")
        if self.pressure_variance < 0.0:
            raise ValueError("pressure_variance must be non-negative")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in the range 1..65535")

        self._client = None
        self._next_connect_time = 0.0
        self._read_failures = 0
        self._sample_logged = False
        self._shutting_down = False
        self._rpc_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="airsim-baro-rpc",
        )
        self._connection_future: Future | None = None
        self._read_future: Future | None = None
        self._pressure_publisher = self.create_publisher(
            FluidPressure,
            self.pressure_topic,
            qos_profile_sensor_data,
        )
        self._altitude_publisher = self.create_publisher(
            Float64,
            self.altitude_topic,
            10,
        )
        self._qnh_publisher = self.create_publisher(
            Float64,
            self.qnh_topic,
            10,
        )
        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f"AirSim barometer bridge started for {self.host}:{self.port}; "
            f"pressure={self.pressure_topic}, frame={self.frame_id}"
        )

    def _ensure_connected(self) -> bool:
        if self._client is not None:
            return True
        now = time.monotonic()
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
                    "python3 -m pip install -r "
                    "~/hutb_ws/src/airsim_ros2_barometer/requirements.txt",
                    throttle_duration_sec=5.0,
                )
                return False
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot connect to AirSim at {self.host}:{self.port}: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False
            self._client = client
            self._read_failures = 0
            self.get_logger().info("Connected to AirSim")
            return True
        if now < self._next_connect_time:
            return False
        self._next_connect_time = now + self.reconnect_period
        self._connection_future = self._rpc_executor.submit(
            self._connect_once
        )
        return False

    def _connect_once(self):
        """Connect outside the ROS executor with a bounded RPC timeout."""
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

    def destroy_node(self) -> None:
        self._shutting_down = True
        self._rpc_executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()

    def _on_timer(self) -> None:
        if self._shutting_down:
            return
        if not self._ensure_connected():
            return
        if self._client is None:
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
        return client.getBarometerData(
            barometer_name=self.barometer_name,
            vehicle_name=self.vehicle_name,
        )

    def _complete_read(self) -> None:
        read_future = self._read_future
        if read_future is None or not read_future.done():
            return
        self._read_future = None
        try:
            sample = read_future.result()
            self._publish_sample(sample)
            self._read_failures = 0
        except Exception as exc:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._client = None
                self._read_failures = 0
                self._next_connect_time = (
                    time.monotonic() + self.reconnect_period
                )
                self.get_logger().warning(
                    f"AirSim barometer connection lost: {exc}"
                )
            else:
                self.get_logger().warning(
                    f"AirSim barometer read failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _publish_sample(self, sample) -> None:
        normalized = parse_barometer_sample(sample)
        if not self._sample_logged:
            self._sample_logged = True
            self.get_logger().info(
                "First barometer sample: "
                f"altitude={type(sample.altitude).__name__}:"
                f"{normalized.altitude:.3f}, "
                f"pressure={type(sample.pressure).__name__}:"
                f"{normalized.pressure:.3f}, "
                f"qnh={type(sample.qnh).__name__}:"
                f"{normalized.qnh_pa:.3f} Pa"
            )
        stamp = self.get_clock().now().to_msg()

        pressure = FluidPressure()
        pressure.header.stamp = stamp
        pressure.header.frame_id = self.frame_id
        pressure.fluid_pressure = normalized.pressure
        pressure.variance = self.pressure_variance
        self._pressure_publisher.publish(pressure)

        altitude = Float64()
        altitude.data = normalized.altitude
        self._altitude_publisher.publish(altitude)

        qnh = Float64()
        qnh.data = normalized.qnh_pa
        self._qnh_publisher.publish(qnh)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AirSimBarometerBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
