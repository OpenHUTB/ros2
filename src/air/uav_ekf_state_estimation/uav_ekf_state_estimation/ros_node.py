"""Optional ROS 2 adapter for the navigation EKF."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String

from .attitude import ComplementaryAttitude
from .ekf import NavigationEKF
from .frames import step_imu


class EkfStateEstimationNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_ekf_state_estimation")
        self.declare_parameter("gps_sigma_m", 1.8)
        self.declare_parameter("baro_sigma_m", 0.35)
        self.ekf: NavigationEKF | None = None
        self.attitude = ComplementaryAttitude()
        self.last_stamp = None
        self.last_imu = None
        self.origin_lat = None
        self.origin_lon = None

        self.create_subscription(Imu, "/uav/imu", self.on_imu, 20)
        self.create_subscription(NavSatFix, "/uav/gps", self.on_gps, 5)
        self.create_subscription(Float64, "/uav/baro", self.on_baro, 10)
        self.odom_pub = self.create_publisher(Odometry, "/uav/ekf/odom", 10)
        self.diag_pub = self.create_publisher(String, "/uav/ekf/diagnostics", 10)

    def on_imu(self, message: Imu) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            self.last_imu = message
            return
        dt = max(1e-4, stamp - self.last_stamp)
        self.last_stamp = stamp
        self.last_imu = message
        # REP-103 body frame (FLU): at rest the specific force is (0, 0, +g).
        accel = np.array(
            [message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
        )
        gyro = np.array(
            [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
        )
        mag = np.array([1.0, 0.0, 0.0])
        # One shared chain: attitude keeps FLU, the EKF receives FRD quantities.
        step_imu(self.attitude, self.ekf, accel, gyro, mag, dt)
        if self.ekf is not None:
            self.publish_odom(message.header.stamp)

    def on_gps(self, message: NavSatFix) -> None:
        if np.isnan(message.altitude):
            return
        if self.origin_lat is None:
            self.origin_lat = float(message.latitude)
            self.origin_lon = float(message.longitude)
            self.ekf = NavigationEKF(np.array([0.0, 0.0, -float(message.altitude)]))
            return
        east = np.radians(message.longitude - self.origin_lon) * 6371000.0 * np.cos(
            np.radians(self.origin_lat)
        )
        north = np.radians(message.latitude - self.origin_lat) * 6371000.0
        sigma = float(self.get_parameter("gps_sigma_m").value)
        self.ekf.update_position(np.array([east, north]), sigma)

    def on_baro(self, message: Float64) -> None:
        if self.ekf is not None:
            self.ekf.update_altitude(-message.data, float(self.get_parameter("baro_sigma_m").value))

    def publish_odom(self, stamp) -> None:
        if self.ekf is None:
            return
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "base_link"
        position = self.ekf.position
        message.pose.pose.position.x = float(position[1])
        message.pose.pose.position.y = float(position[0])
        message.pose.pose.position.z = float(-position[2])
        velocity = self.ekf.velocity
        message.twist.twist.linear.x = float(velocity[1])
        message.twist.twist.linear.y = float(velocity[0])
        message.twist.twist.linear.z = float(-velocity[2])
        self.odom_pub.publish(message)
        diagnostics = {
            "position_ned": [round(float(value), 4) for value in position],
            "velocity_ned": [round(float(value), 4) for value in self.ekf.velocity],
        }
        self.diag_pub.publish(String(data=json.dumps(diagnostics)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfStateEstimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
