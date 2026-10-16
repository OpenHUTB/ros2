"""ROS 2 entry point for the MuJoCo differential-drive simulation."""

from pathlib import Path
from typing import Any

from .kinematics import CommandWatchdog, DriveGeometry
from .resources import resolve_package_file
from .simulator import MujocoSimulation


def _load_ros_dependencies() -> dict[str, Any]:
    """Import ROS modules only when the executable is launched."""
    try:
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from tf2_ros import TransformBroadcaster
    except ImportError as exc:
        raise RuntimeError(
            '缺少 ROS 2 Humble Python 依赖 / ROS 2 Humble Python dependencies are required'
        ) from exc
    return {
        'rclpy': rclpy,
        'get_package_share_directory': get_package_share_directory,
        'TransformStamped': TransformStamped,
        'Twist': Twist,
        'Odometry': Odometry,
        'Node': Node,
        'Clock': Clock,
        'TransformBroadcaster': TransformBroadcaster,
    }


def _build_node_class(deps: dict[str, Any]) -> type:
    Node = deps['Node']
    Twist = deps['Twist']
    Odometry = deps['Odometry']
    Clock = deps['Clock']
    TransformStamped = deps['TransformStamped']
    TransformBroadcaster = deps['TransformBroadcaster']
    get_package_share_directory = deps['get_package_share_directory']

    class SimulationNode(Node):
        """Advance MuJoCo at a fixed rate and expose ROS velocity control."""

        def __init__(self) -> None:
            super().__init__('mujoco_diff_drive_simulation')
            self.declare_parameter('model_file', 'models/diff_drive.xml')
            self.declare_parameter('wheel_radius', 0.08)
            self.declare_parameter('track_width', 0.42)
            self.declare_parameter('max_wheel_speed', 20.0)
            self.declare_parameter('command_timeout', 0.5)
            self.declare_parameter('publish_tf', True)

            package_share = Path(
                get_package_share_directory(
                    'mujoco_diff_drive_slip_compensation'
                )
            )
            model_path = resolve_package_file(
                package_share,
                str(self.get_parameter('model_file').value),
            )
            self._geometry = DriveGeometry(
                wheel_radius=float(self.get_parameter('wheel_radius').value),
                track_width=float(self.get_parameter('track_width').value),
                max_wheel_speed=float(
                    self.get_parameter('max_wheel_speed').value
                ),
            )
            self._watchdog = CommandWatchdog(
                float(self.get_parameter('command_timeout').value)
            )
            self._simulation = MujocoSimulation(model_path)
            self._publish_tf = bool(self.get_parameter('publish_tf').value)

            self._odom_publisher = self.create_publisher(
                Odometry, 'ground_truth/odom', 10
            )
            self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
            self._tf_broadcaster = TransformBroadcaster(self)
            self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
            self.create_timer(self._simulation.timestep, self._on_timer)
            self.get_logger().info(
                'MuJoCo 差速仿真已启动 / MuJoCo differential-drive simulation started'
            )

        def _on_command(self, message: Any) -> None:
            try:
                self._watchdog.update(
                    float(message.linear.x), float(message.angular.z)
                )
            except ValueError as exc:
                self.get_logger().error(
                    f'拒绝无效速度指令 / rejected invalid velocity command: {exc}'
                )

        def _on_timer(self) -> None:
            linear, angular = self._watchdog.current()
            left, right = self._geometry.wheel_speeds(linear, angular)
            self._simulation.apply_wheel_speeds(left, right)
            self._simulation.step()
            self._publish_state()

        def _publish_state(self) -> None:
            state = self._simulation.planar_state()
            quaternion = self._simulation.chassis_quaternion()
            timestamp = self.get_clock().now().to_msg()

            odometry = Odometry()
            odometry.header.stamp = timestamp
            odometry.header.frame_id = 'odom'
            odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = state.x
            odometry.pose.pose.position.y = state.y
            odometry.pose.pose.orientation.w = quaternion[0]
            odometry.pose.pose.orientation.x = quaternion[1]
            odometry.pose.pose.orientation.y = quaternion[2]
            odometry.pose.pose.orientation.z = quaternion[3]
            odometry.twist.twist.linear.x = state.linear_velocity
            odometry.twist.twist.angular.z = state.angular_velocity
            self._odom_publisher.publish(odometry)

            clock = Clock()
            seconds = int(state.time)
            clock.clock.sec = seconds
            clock.clock.nanosec = int((state.time - seconds) * 1_000_000_000)
            self._clock_publisher.publish(clock)

            if self._publish_tf:
                transform = TransformStamped()
                transform.header = odometry.header
                transform.child_frame_id = odometry.child_frame_id
                transform.transform.translation.x = state.x
                transform.transform.translation.y = state.y
                transform.transform.rotation = odometry.pose.pose.orientation
                self._tf_broadcaster.sendTransform(transform)

    return SimulationNode


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 simulation node."""
    deps = _load_ros_dependencies()
    rclpy = deps['rclpy']
    node = None
    rclpy.init(args=args)
    try:
        node = _build_node_class(deps)()
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
