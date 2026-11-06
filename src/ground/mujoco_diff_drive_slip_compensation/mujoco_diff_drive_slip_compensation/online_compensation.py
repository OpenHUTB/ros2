"""ROS 2 adapter for online slip inference and velocity compensation."""

from pathlib import Path
from typing import Any

from .compensation import SlipCompensationPolicy
from .slip_model import TemporalSlipMlp


def _load_ros_dependencies() -> dict[str, Any]:
    """Import ROS modules only when the executable is launched."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float64, String
    except ImportError as exc:
        raise RuntimeError(
            'ROS 2 Humble Python dependencies are required'
        ) from exc
    return {
        'rclpy': rclpy,
        'Twist': Twist,
        'Odometry': Odometry,
        'Node': Node,
        'Imu': Imu,
        'JointState': JointState,
        'Float64': Float64,
        'String': String,
    }


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _build_node_class(deps: dict[str, Any]) -> type:
    Node = deps['Node']
    Twist = deps['Twist']
    Odometry = deps['Odometry']
    Imu = deps['Imu']
    JointState = deps['JointState']
    Float64 = deps['Float64']
    String = deps['String']

    class OnlineCompensationNode(Node):
        """Consume synchronized ROS measurements and publish safe commands."""

        WHEEL_NAMES = (
            'left_front_joint', 'left_rear_joint',
            'right_front_joint', 'right_rear_joint',
        )

        def __init__(self) -> None:
            super().__init__('online_slip_compensation')
            self.declare_parameter(
                'model_file', 'data/models/slip-v1/slip_model.json'
            )
            self.declare_parameter('sensor_timeout_seconds', 0.2)
            self.declare_parameter('max_linear_scale', 1.5)
            self.declare_parameter('max_linear_acceleration', 1.5)
            self.declare_parameter('max_angular_acceleration', 3.0)
            model_path = Path(str(self.get_parameter('model_file').value))
            if not model_path.is_file():
                raise FileNotFoundError(f'slip model not found: {model_path}')
            model = TemporalSlipMlp.load(model_path)
            self._policy = SlipCompensationPolicy(
                model,
                sensor_timeout_seconds=float(
                    self.get_parameter('sensor_timeout_seconds').value
                ),
                max_linear_scale=float(
                    self.get_parameter('max_linear_scale').value
                ),
                max_linear_acceleration=float(
                    self.get_parameter('max_linear_acceleration').value
                ),
                max_angular_acceleration=float(
                    self.get_parameter('max_angular_acceleration').value
                ),
            )
            self._imu: Any | None = None
            self._odometry: Any | None = None
            self._command: Any | None = None
            self._command_stamp_ns = 0
            self._raw_publisher = self.create_publisher(Twist, 'cmd_vel/raw', 10)
            self._command_publisher = self.create_publisher(
                Twist, 'cmd_vel/compensated', 10
            )
            self._slip_publisher = self.create_publisher(
                Float64, 'slip/estimate', 10
            )
            self._diagnostic_publisher = self.create_publisher(
                String, 'slip/status', 10
            )
            self.create_subscription(
                Twist, 'cmd_vel', self._on_command, 10
            )
            self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
            self.create_subscription(
                Odometry, 'ground_truth/odom', self._on_odometry, 10
            )
            self.create_subscription(
                JointState, 'wheel/encoders', self._on_encoders, 10
            )
            self.get_logger().info(
                f'Loaded slip model / 已加载滑移模型: {model_path}'
            )

        def _on_command(self, message: Any) -> None:
            self._command = message
            self._command_stamp_ns = self.get_clock().now().nanoseconds

        def _on_imu(self, message: Any) -> None:
            self._imu = message

        def _on_odometry(self, message: Any) -> None:
            self._odometry = message

        def _publish_stop(self, reason: str) -> None:
            command = Twist()
            self._command_publisher.publish(command)
            status = String()
            status.data = reason
            self._diagnostic_publisher.publish(status)

        def _on_encoders(self, message: Any) -> None:
            names = list(message.name)
            try:
                indices = [names.index(name) for name in self.WHEEL_NAMES]
                wheels = tuple(float(message.velocity[index]) for index in indices)
            except (ValueError, IndexError):
                self._publish_stop('encoder wheel names or velocities invalid')
                return
            if self._imu is None or self._odometry is None or self._command is None:
                self._publish_stop('waiting for IMU, odometry, and command')
                return
            now_ns = _stamp_ns(message)
            acceleration = self._imu.linear_acceleration
            angular_velocity = self._imu.angular_velocity
            command = self._command
            self._raw_publisher.publish(command)
            result = self._policy.update(
                stamp_ns=now_ns,
                imu_stamp_ns=_stamp_ns(self._imu),
                odometry_stamp_ns=_stamp_ns(self._odometry),
                command_stamp_ns=self._command_stamp_ns,
                wheel_velocities=wheels,
                acceleration=(
                    float(acceleration.x), float(acceleration.y),
                    float(acceleration.z),
                ),
                angular_velocity=(
                    float(angular_velocity.x), float(angular_velocity.y),
                    float(angular_velocity.z),
                ),
                command_linear=float(command.linear.x),
                command_angular=float(command.angular.z),
            )
            output = Twist()
            output.linear.x = result.linear
            output.angular.z = result.angular
            self._command_publisher.publish(output)
            slip = Float64()
            slip.data = result.slip
            self._slip_publisher.publish(slip)
            status = String()
            status.data = result.reason
            self._diagnostic_publisher.publish(status)

    return OnlineCompensationNode


def main(args: list[str] | None = None) -> None:
    """Run the online slip-compensation ROS node."""
    deps = _load_ros_dependencies()
    rclpy = deps['rclpy']
    node = _build_node_class(deps)()
    rclpy.init(args=args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
