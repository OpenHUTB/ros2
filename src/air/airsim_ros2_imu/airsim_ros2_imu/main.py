"""AirSim IMU bridge for ROS 2 Humble."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu import (
    SourceTimestampTracker,
    airsim_stamp_to_time,
    build_imu_message,
    validate_parameters,
)


class AirSimImuBridge(Node):
    """Publish AirSim IMU samples using ROS REP-103 conventions."""

    def __init__(self) -> None:
        super().__init__("airsim_imu_bridge")
        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 41451).value)
        self.vehicle_name = str(
            self.declare_parameter("vehicle_name", "").value
        )
        self.imu_name = str(self.declare_parameter("imu_name", "").value)
        self.imu_topic = str(
            self.declare_parameter("imu_topic", "/uav/imu").value
        )
        self.frame_id = str(
            self.declare_parameter("frame_id", "base_link").value
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 50.0).value
        )
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        self.stale_timeout = float(
            self.declare_parameter("stale_timeout", 0.25).value
        )
        self.use_source_timestamp = bool(
            self.declare_parameter("use_source_timestamp", True).value
        )
        self.orientation_variance = float(
            self.declare_parameter("orientation_variance", 0.001).value
        )
        self.angular_velocity_variance = float(
            self.declare_parameter("angular_velocity_variance", 0.0001).value
        )
        self.linear_acceleration_variance = float(
            self.declare_parameter("linear_acceleration_variance", 0.01).value
        )

        validate_parameters(
            port=self.port,
            publish_rate=self.publish_rate,
            reconnect_period=self.reconnect_period,
            stale_timeout=self.stale_timeout,
            orientation_variance=self.orientation_variance,
            angular_velocity_variance=self.angular_velocity_variance,
            linear_acceleration_variance=self.linear_acceleration_variance,
        )

        self._client = None
        self._next_connect_time = 0.0
        self._read_failures = 0
        self._source_tracker = SourceTimestampTracker()
        self._stale_reported = False
        self._imu_publisher = self.create_publisher(
            Imu,
            self.imu_topic,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f"AirSim IMU bridge started for {self.host}:{self.port}; "
            f"topic={self.imu_topic}, frame={self.frame_id}"
        )

    def _ensure_connected(self) -> bool:
        if self._client is not None:
            return True
        now = time.monotonic()
        if now < self._next_connect_time:
            return False
        self._next_connect_time = now + self.reconnect_period
        try:
            import airsim

            client = airsim.MultirotorClient(ip=self.host, port=self.port)
            client.confirmConnection()
            self._client = client
            self._read_failures = 0
            self._source_tracker.reset()
            self._stale_reported = False
            self.get_logger().info("Connected to AirSim")
            return True
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot connect to AirSim at {self.host}:{self.port}: {exc}",
                throttle_duration_sec=5.0,
            )
            return False

    def _on_timer(self) -> None:
        if not self._ensure_connected():
            return
        assert self._client is not None
        try:
            sample = self._client.getImuData(
                imu_name=self.imu_name,
                vehicle_name=self.vehicle_name,
            )
            now = time.monotonic()
            source_stamp = int(getattr(sample, "time_stamp", 0))
            if not self._source_tracker.observe(source_stamp, now):
                if (
                    self._source_tracker.age(now) >= self.stale_timeout
                    and not self._stale_reported
                ):
                    self.get_logger().warning(
                        "AirSim IMU source timestamp is stale; "
                        "suppressing repeated samples"
                    )
                    self._stale_reported = True
                return
            if self._stale_reported:
                self.get_logger().info("AirSim IMU source timestamp recovered")
                self._stale_reported = False
            stamp = (
                airsim_stamp_to_time(source_stamp)
                if self.use_source_timestamp and source_stamp > 0
                else self.get_clock().now().to_msg()
            )
            self._publish_sample(sample, stamp)
            self._read_failures = 0
        except Exception as exc:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._client = None
                self._read_failures = 0
                self._source_tracker.reset()
                self.get_logger().warning(f"AirSim IMU connection lost: {exc}")
            else:
                self.get_logger().warning(
                    f"AirSim IMU read failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _publish_sample(self, sample, stamp) -> None:
        self._imu_publisher.publish(
            build_imu_message(
                sample,
                frame_id=self.frame_id,
                stamp=stamp,
                orientation_variance=self.orientation_variance,
                angular_velocity_variance=self.angular_velocity_variance,
                linear_acceleration_variance=self.linear_acceleration_variance,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirSimImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
