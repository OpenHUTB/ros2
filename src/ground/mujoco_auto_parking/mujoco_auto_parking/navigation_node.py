"""ROS 2 node: online mapping, Neural A* planning and neural path following."""

import math
import time
from pathlib import Path as FilePath

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .navigation import NavigationCore
from .resources import resolve_package_file
from .simulator import SimulationState


class NavigationNode(Node):
    """Build a map from /scan, plan with Neural A*, and follow with the NN."""

    def __init__(self) -> None:
        super().__init__('navigation_node')
        self.declare_parameter('planner_weights', 'models/planner.npz')
        self.declare_parameter('controller_weights', 'models/trajectory.npz')
        self.declare_parameter('goal', '0.25,1.2')
        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('map_publish_period', 1.0)

        package_share = FilePath(
            get_package_share_directory('mujoco_auto_parking')
        )
        planner_weights = resolve_package_file(
            package_share, str(self.get_parameter('planner_weights').value)
        )
        controller_weights = resolve_package_file(
            package_share, str(self.get_parameter('controller_weights').value)
        )
        goal_text = str(self.get_parameter('goal').value)
        parts = goal_text.split(',')
        if len(parts) != 2:
            raise ValueError('goal must be "x,y"')
        self._goal = (float(parts[0]), float(parts[1]))
        self._replan_period = float(self.get_parameter('replan_period').value)
        self._map_publish_period = float(
            self.get_parameter('map_publish_period').value
        )
        self._core = NavigationCore(planner_weights, controller_weights)

        self._state = None
        self._scan = None
        self._finished = False
        self._last_map_publish = -1e9
        self._publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._publisher_map = self.create_publisher(
            OccupancyGrid, '/map', 10
        )
        self._publisher_path = self.create_publisher(Path, '/global_path', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        control_rate = float(self.get_parameter('control_rate').value)
        if control_rate <= 0.0:
            raise ValueError('control_rate must be positive')
        self.create_timer(1.0 / control_rate, self._on_timer)
        self.get_logger().info(
            f'导航节点已启动，目标 {self._goal}，'
            f'规划权值 {planner_weights}'
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
        self._state = SimulationState(
            time=0.0,
            x=float(position.x),
            y=float(position.y),
            yaw=yaw,
            linear_velocity=float(message.twist.twist.linear.x),
            angular_velocity=float(message.twist.twist.angular.z),
        )

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message

    def _on_timer(self) -> None:
        command = Twist()
        if self._state is None or self._scan is None or self._finished:
            self._publisher_cmd.publish(command)
            return

        self._core.update_map(self._state, self._scan)
        now = time.monotonic()
        if now - self._core.last_plan_time >= self._replan_period:
            self._core.plan(self._state, self._goal)
            self._publish_path()
            self.get_logger().info(
                f'已规划：{self._core.last_planning_stats}, '
                f'地图覆盖 {self._core.map.statistics()["coverage_pct"]:.1f}%'
            )

        speed, yaw_rate = self._core.control(self._state)
        if self._scan.ranges and min(self._scan.ranges) < 0.35:
            speed = 0.0
        command.linear.x = float(speed)
        command.angular.z = float(yaw_rate)
        self._publisher_cmd.publish(command)

        distance = math.hypot(
            self._state.x - self._goal[0], self._state.y - self._goal[1]
        )
        if distance < 0.35:
            self._finished = True
            self._publisher_cmd.publish(Twist())
            self.get_logger().info('导航到达目标，速度归零')

        if now - self._last_map_publish >= self._map_publish_period:
            self._publish_map()
            self._last_map_publish = now

    def _publish_map(self) -> None:
        grid = self._core.map.to_ros_grid()
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = self._core.map.resolution
        message.info.width = self._core.map.width
        message.info.height = self._core.map.height
        message.info.origin.position.x = self._core.map.origin[0]
        message.info.origin.position.y = self._core.map.origin[1]
        message.info.origin.orientation.w = 1.0
        message.data = grid.flatten().astype(np.int8).tolist()
        self._publisher_map.publish(message)

    def _publish_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        for index, (x, y) in enumerate(self._core.path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            if index < len(self._core.path) - 1:
                nx, ny = self._core.path[index + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = self._core.path[index - 1]
                yaw = math.atan2(y - py, x - px)
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        self._publisher_path.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationNode()
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
