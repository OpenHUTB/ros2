"""AirSim GNSS bridge for ROS 2 Humble."""

from __future__ import annotations

import time
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .gnss import navsat_status, parse_gnss_sample, position_covariance


class AirSimGnssBridge(Node):
    """Publish AirSim GNSS samples as ROS NavSatFix messages."""

    def __init__(self) -> None:
        super().__init__("airsim_gnss_bridge")
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 41451).value)
        self.vehicle_name = str(
            self.declare_parameter("vehicle_name", "").value
        )
        self.gps_name = str(self.declare_parameter("gps_name", "").value)
        self.fix_topic = str(
            self.declare_parameter("fix_topic", "/uav/gps/fix").value
        )
        self.frame_id = str(
            self.declare_parameter("frame_id", "gps_link").value
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 5.0).value
        )
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        self.connection_timeout = float(
            self.declare_parameter("connection_timeout", 1.0).value
        )
        self.minimum_fix_type = int(
            self.declare_parameter("minimum_fix_type", 2).value
        )
        self.fallback_horizontal_variance = float(
            self.declare_parameter(
                "fallback_horizontal_variance",
                25.0,
            ).value
        )
        self.fallback_vertical_variance = float(
            self.declare_parameter(
                "fallback_vertical_variance",
                49.0,
            ).value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.reconnect_period <= 0.0:
            raise ValueError("reconnect_period must be positive")
        if self.connection_timeout <= 0.0:
            raise ValueError("connection_timeout must be positive")
        if self.minimum_fix_type < 0:
            raise ValueError("minimum_fix_type must be non-negative")

        self._client = None
        self._next_connect_time = 0.0
        self._read_failures = 0
        self._connection_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="airsim-gnss-connect",
        )
        self._connection_future: Future | None = None
        self._fix_publisher = self.create_publisher(
            NavSatFix,
            self.fix_topic,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f"AirSim GNSS bridge started for {self.host}:{self.port}; "
            f"topic={self.fix_topic}, frame={self.frame_id}"
        )

    def _ensure_connected(self) -> bool:
        if self._client is not None:
            return True
        now = time.monotonic()
        if self._connection_future is not None:
            if not self._connection_future.done():
                return False
            try:
                client = self._connection_future.result()
            except Exception as exc:
                self.get_logger().warning(
                    f"Cannot connect to AirSim at {self.host}:{self.port}: {exc}",
                    throttle_duration_sec=5.0,
                )
                return False
            finally:
                self._connection_future = None
            self._client = client
            self._read_failures = 0
            self.get_logger().info("Connected to AirSim")
            return True
        if now < self._next_connect_time:
            return False
        self._next_connect_time = now + self.reconnect_period
        self._connection_future = self._connection_executor.submit(
            self._connect_once
        )
        return False

    def _connect_once(self):
        """Connect off the ROS executor thread with a bounded RPC timeout."""
        import airsim

        client = airsim.MultirotorClient(
            ip=self.host,
            port=self.port,
            timeout_value=self.connection_timeout,
        )
        client.confirmConnection()
        return client

    def destroy_node(self) -> None:
        self._connection_executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()

    def _on_timer(self) -> None:
        if not self._ensure_connected():
            return
        assert self._client is not None
        try:
            sample = self._client.getGpsData(
                gps_name=self.gps_name,
                vehicle_name=self.vehicle_name,
            )
            self._publish_sample(sample)
            self._read_failures = 0
        except Exception as exc:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._client = None
                self._read_failures = 0
                self.get_logger().warning(f"AirSim GNSS connection lost: {exc}")
            else:
                self.get_logger().warning(
                    f"AirSim GNSS read failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _publish_sample(self, sample) -> None:
        normalized = parse_gnss_sample(sample)
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.status.service = NavSatStatus.SERVICE_GPS
        message.status.status = navsat_status(
            normalized,
            self.minimum_fix_type,
        )
        message.latitude = normalized.latitude
        message.longitude = normalized.longitude
        message.altitude = normalized.altitude
        covariance = position_covariance(
            normalized,
            self.fallback_horizontal_variance,
            self.fallback_vertical_variance,
        )
        message.position_covariance[0] = covariance[0]
        message.position_covariance[4] = covariance[1]
        message.position_covariance[8] = covariance[2]
        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )
        self._fix_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirSimGnssBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
