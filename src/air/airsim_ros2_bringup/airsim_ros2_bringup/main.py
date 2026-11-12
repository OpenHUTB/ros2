"""Main entry for the minimal AirSim ROS 2 Humble bridge."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .transforms import (
    airsim_quat_to_ros_quat_flu,
    body_flu_to_airsim,
    enu_to_ned,
    ned_to_enu,
    world_enu_to_body_flu,
)


class AirSimRos2Bridge(Node):
    """Publish AirSim odometry/TF and forward the latest velocity command."""

    def __init__(self) -> None:
        super().__init__("airsim_ros2_bridge")

        self.host = str(self.declare_parameter("host", "127.0.0.1").value)
        self.port = int(self.declare_parameter("port", 41451).value)
        self.vehicle_name = str(self.declare_parameter("vehicle_name", "").value)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/uav/cmd_vel").value
        )
        self.odom_topic = str(
            self.declare_parameter("odom_topic", "/uav/odom").value
        )
        self.cmd_frame = str(
            self.declare_parameter("cmd_vel_frame", "body_flu").value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.5).value
        )
        self.command_duration = float(
            self.declare_parameter("command_duration", 0.2).value
        )
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        self.takeoff_timeout = float(
            self.declare_parameter("takeoff_timeout", 20.0).value
        )
        self.land_timeout = float(self.declare_parameter("land_timeout", 20.0).value)

        if self.cmd_frame not in ("body_flu", "world_enu"):
            raise ValueError("cmd_vel_frame must be 'body_flu' or 'world_enu'")
        if self.publish_rate <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("publish_rate and command_timeout must be positive")

        self._airsim = None
        self._client = None
        self._next_connect_time = 0.0
        self._last_command: Optional[Twist] = None
        self._last_command_time = 0.0
        self._hover_sent = True
        self._read_failures = 0

        self._odom_publisher = self.create_publisher(
            Odometry, self.odom_topic, qos_profile_sensor_data
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_command, qos_profile_sensor_data
        )
        self.create_service(
            Trigger, "/uav/takeoff", self._handle_takeoff
        )
        self.create_service(Trigger, "/uav/land", self._handle_land)
        self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f"AirSim ROS 2 bridge started for {self.host}:{self.port}; "
            f"world={self.world_frame}, base={self.base_frame}, "
            f"commands={self.cmd_vel_topic} ({self.cmd_frame})"
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
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            self._airsim = airsim
            self._client = client
            self._read_failures = 0
            self.get_logger().info("Connected to AirSim")
            return True
        except Exception as exc:
            self.get_logger().warning(
                f"Cannot connect to AirSim at {self.host}:{self.port}: {exc}",
                throttle_duration_sec=5.0,
            )
            return False

    def _drop_connection(self, reason: str) -> None:
        if self._client is not None:
            self.get_logger().warning(f"AirSim connection lost: {reason}")
        self._client = None
        self._last_command = None
        self._hover_sent = True

    def _on_command(self, message: Twist) -> None:
        self._last_command = message
        self._last_command_time = time.monotonic()
        self._hover_sent = False

    def _on_timer(self) -> None:
        if not self._ensure_connected():
            return
        assert self._client is not None
        try:
            state = self._client.getMultirotorState(
                vehicle_name=self.vehicle_name
            )
            self._publish_odometry(state)
            self._apply_command()
        except Exception as exc:
            self._read_failures += 1
            if self._read_failures >= 5:
                self._drop_connection(str(exc))
            else:
                self.get_logger().warning(
                    f"AirSim telemetry read failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _publish_odometry(self, state) -> None:
        kinematics = state.kinematics_estimated
        position = ned_to_enu(
            (
                kinematics.position.x_val,
                kinematics.position.y_val,
                kinematics.position.z_val,
            )
        )
        orientation = airsim_quat_to_ros_quat_flu(
            (
                kinematics.orientation.w_val,
                kinematics.orientation.x_val,
                kinematics.orientation.y_val,
                kinematics.orientation.z_val,
            )
        )
        velocity_world = ned_to_enu(
            (
                kinematics.linear_velocity.x_val,
                kinematics.linear_velocity.y_val,
                kinematics.linear_velocity.z_val,
            )
        )
        angular_world = ned_to_enu(
            (
                kinematics.angular_velocity.x_val,
                kinematics.angular_velocity.y_val,
                kinematics.angular_velocity.z_val,
            )
        )
        velocity_body = world_enu_to_body_flu(velocity_world, orientation)
        angular_body = world_enu_to_body_flu(angular_world, orientation)
        stamp = self.get_clock().now().to_msg()

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.world_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = position[0]
        odometry.pose.pose.position.y = position[1]
        odometry.pose.pose.position.z = position[2]
        odometry.pose.pose.orientation.x = orientation[0]
        odometry.pose.pose.orientation.y = orientation[1]
        odometry.pose.pose.orientation.z = orientation[2]
        odometry.pose.pose.orientation.w = orientation[3]
        odometry.twist.twist.linear.x = velocity_body[0]
        odometry.twist.twist.linear.y = velocity_body[1]
        odometry.twist.twist.linear.z = velocity_body[2]
        odometry.twist.twist.angular.x = angular_body[0]
        odometry.twist.twist.angular.y = angular_body[1]
        odometry.twist.twist.angular.z = angular_body[2]

        for index in (0, 7, 14):
            odometry.pose.covariance[index] = 0.01
            odometry.twist.covariance[index] = 0.01
        for index in (21, 28, 35):
            odometry.pose.covariance[index] = 0.1
            odometry.twist.covariance[index] = 0.1
        self._odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        self._tf_broadcaster.sendTransform(transform)

    def _apply_command(self) -> None:
        assert self._airsim is not None
        assert self._client is not None
        age = time.monotonic() - self._last_command_time
        if self._last_command is None or age > self.command_timeout:
            if not self._hover_sent:
                self._client.hoverAsync(vehicle_name=self.vehicle_name)
                self._hover_sent = True
                self.get_logger().info("Command timeout; AirSim is hovering")
            return

        command = self._last_command
        yaw_mode = self._airsim.YawMode(
            is_rate=True, yaw_or_rate=math.degrees(command.angular.z)
        )
        vector = (command.linear.x, command.linear.y, command.linear.z)
        if self.cmd_frame == "body_flu":
            body_ned = body_flu_to_airsim(vector)
            self._client.moveByVelocityBodyFrameAsync(
                body_ned[0],
                body_ned[1],
                body_ned[2],
                self.command_duration,
                yaw_mode=yaw_mode,
                vehicle_name=self.vehicle_name,
            )
        else:
            world_ned = enu_to_ned(vector)
            self._client.moveByVelocityAsync(
                world_ned[0],
                world_ned[1],
                world_ned[2],
                self.command_duration,
                yaw_mode=yaw_mode,
                vehicle_name=self.vehicle_name,
            )

    def _handle_takeoff(self, request, response):
        del request
        if not self._ensure_connected():
            response.success = False
            response.message = "AirSim is not connected"
            return response
        try:
            self._client.armDisarm(True, vehicle_name=self.vehicle_name)
            self._client.takeoffAsync(
                timeout_sec=self.takeoff_timeout,
                vehicle_name=self.vehicle_name,
            ).join()
            response.success = True
            response.message = "Takeoff completed"
        except Exception as exc:
            response.success = False
            response.message = f"Takeoff failed: {exc}"
        return response

    def _handle_land(self, request, response):
        del request
        if not self._ensure_connected():
            response.success = False
            response.message = "AirSim is not connected"
            return response
        try:
            self._client.hoverAsync(vehicle_name=self.vehicle_name)
            self._client.landAsync(
                timeout_sec=self.land_timeout,
                vehicle_name=self.vehicle_name,
            ).join()
            response.success = True
            response.message = "Landing completed"
        except Exception as exc:
            response.success = False
            response.message = f"Landing failed: {exc}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirSimRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
