"""ROS 2 node: neural control along a given S-curve trajectory."""

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .resources import resolve_package_file
from .trajectory_control import (
    ReferenceTrajectory,
    TrajectoryController,
    build_s_curve,
    wrap_angle,
)


class TrajectoryControlNode(Node):
    """Track a reference S-curve with the trained control MLP."""

    def __init__(self) -> None:
        super().__init__('trajectory_control_node')
        self.declare_parameter(
            'weights_file', 'models/trajectory.npz'
        )
        self.declare_parameter('trajectory', 's_curve')
        self.declare_parameter('v_ref', 0.45)
        self.declare_parameter('control_rate', 20.0)

        package_share = Path(
            get_package_share_directory('mujoco_auto_parking')
        )
        weights = resolve_package_file(
            package_share, str(self.get_parameter('weights_file').value)
        )
        self._controller = TrajectoryController(weights)
        self._v_ref = float(self.get_parameter('v_ref').value)
        control_rate = float(self.get_parameter('control_rate').value)
        if control_rate <= 0.0:
            raise ValueError('control_rate must be positive')
        self._trajectory = None
        self._state = None
        self._finished = False
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(1.0 / control_rate, self._on_timer)
        self.get_logger().info(
            f'轨迹控制节点已启动，权值: {weights}, v_ref={self._v_ref:.2f}'
        )

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw = float(math.atan2(sin_yaw, cos_yaw))
        self._state = (
            float(position.x),
            float(position.y),
            yaw,
            float(message.twist.twist.linear.x),
        )
        if self._trajectory is None:
            points = build_s_curve(
                float(position.x), float(position.y), yaw
            )
            self._trajectory = ReferenceTrajectory(points)
            self.get_logger().info(
                '已生成给定轨迹：S 曲线，长度 4.0 m，幅值 1.0 m'
            )

    def _on_timer(self) -> None:
        message = Twist()
        if (
            self._state is None
            or self._trajectory is None
            or self._finished
        ):
            self._publisher.publish(message)
            return

        x, y, yaw, velocity = self._state
        index, distance, reference = self._trajectory.lookahead(x, y)
        speed, yaw_rate = self._controller.predict(
            (x, y, yaw, velocity), reference, self._v_ref
        )
        if index >= len(self._trajectory.points) - 4 and distance < 0.25:
            self._finished = True
            self.get_logger().info('已到达给定轨迹终点，速度归零')
            self._publisher.publish(message)
            return
        message.linear.x = speed
        message.angular.z = yaw_rate
        self._publisher.publish(message)

        heading_error = abs(
            wrap_angle(float(reference[2]) - yaw)
        )
        if heading_error > 1.2:
            message.linear.x = 0.1
            message.angular.z = float(
                max(-1.2, min(1.2, 2.0 * heading_error))
            )
            self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
