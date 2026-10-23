"""ROS 2 entry point for the MuJoCo differential-drive simulation."""

from pathlib import Path
from typing import Any

from .kinematics import CommandWatchdog, DriveGeometry
from .resources import resolve_package_file
from .scenarios import FrictionSchedule
from .sensors import SensorNoise, SensorNoiseConfig
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
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float64, String
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
        'Imu': Imu,
        'JointState': JointState,
        'Float64': Float64,
        'String': String,
        'TransformBroadcaster': TransformBroadcaster,
    }


def _build_node_class(deps: dict[str, Any]) -> type:
    Node = deps['Node']
    Twist = deps['Twist']
    Odometry = deps['Odometry']
    Clock = deps['Clock']
    Imu = deps['Imu']
    JointState = deps['JointState']
    Float64 = deps['Float64']
    String = deps['String']
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
            self.declare_parameter('random_seed', 25012)
            self.declare_parameter('friction_coefficients', [0.25, 0.55, 0.9])
            self.declare_parameter('scenario_duration', 30.0)
            self.declare_parameter('encoder_position_noise_std', 0.0005)
            self.declare_parameter('encoder_velocity_noise_std', 0.005)
            self.declare_parameter('accelerometer_noise_std', 0.02)
            self.declare_parameter('gyroscope_noise_std', 0.002)

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
            self._friction_schedule = FrictionSchedule(
                self.get_parameter('friction_coefficients').value,
                float(self.get_parameter('scenario_duration').value),
            )
            self._noise_config = SensorNoiseConfig(
                encoder_position_std=float(
                    self.get_parameter('encoder_position_noise_std').value
                ),
                encoder_velocity_std=float(
                    self.get_parameter('encoder_velocity_noise_std').value
                ),
                accelerometer_std=float(
                    self.get_parameter('accelerometer_noise_std').value
                ),
                gyroscope_std=float(
                    self.get_parameter('gyroscope_noise_std').value
                ),
            )
            self._sensor_noise = SensorNoise(
                self._noise_config, int(self.get_parameter('random_seed').value)
            )
            self._last_scenario_index = -1
            self._last_episode_number = -1
            self._elapsed_time = 0.0

            self._odom_publisher = self.create_publisher(
                Odometry, 'ground_truth/odom', 10
            )
            self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
            self._encoder_publisher = self.create_publisher(
                JointState, 'wheel/encoders', 10
            )
            self._imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
            self._friction_publisher = self.create_publisher(
                Float64, 'scenario/friction', 10
            )
            self._episode_publisher = self.create_publisher(
                String, 'scenario/episode_id', 10
            )
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
            scenario = self._friction_schedule.at(self._elapsed_time)
            if scenario.episode_number != self._last_episode_number:
                self._simulation.reset(preserve_time=True)
                self._simulation.set_ground_friction(scenario.coefficient)
                self._last_scenario_index = scenario.index
                self._last_episode_number = scenario.episode_number
                self.get_logger().info(
                    '切换摩擦场景 / switched friction scenario: '
                    f'index={scenario.index}, coefficient={scenario.coefficient:.3f}'
                )
                self._publish_scenario(
                    scenario.coefficient, scenario.episode_id
                )
                return
            linear, angular = self._watchdog.current()
            left, right = self._geometry.wheel_speeds(linear, angular)
            self._simulation.apply_wheel_speeds(left, right)
            self._simulation.step()
            self._elapsed_time += self._simulation.timestep
            self._publish_state(scenario.coefficient, scenario.episode_id)

        def _publish_state(
            self, friction_coefficient: float, episode_id: str
        ) -> None:
            self._publish_scenario(friction_coefficient, episode_id)
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

            positions, velocities = self._simulation.wheel_state()
            noisy_positions, noisy_velocities = self._sensor_noise.encoders(
                positions, velocities
            )
            encoders = JointState()
            encoders.header.stamp = timestamp
            encoders.name = list(MujocoSimulation.WHEEL_JOINT_NAMES)
            encoders.position = list(noisy_positions)
            encoders.velocity = list(noisy_velocities)
            self._encoder_publisher.publish(encoders)

            acceleration, angular_velocity = self._simulation.imu_state()
            noisy_acceleration, noisy_angular = self._sensor_noise.imu(
                acceleration, angular_velocity
            )
            imu = Imu()
            imu.header.stamp = timestamp
            imu.header.frame_id = 'imu_link'
            imu.orientation_covariance[0] = -1.0
            (
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ) = noisy_acceleration
            (
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ) = noisy_angular
            acceleration_variance = self._noise_config.accelerometer_std ** 2
            angular_variance = self._noise_config.gyroscope_std ** 2
            for index in (0, 4, 8):
                imu.linear_acceleration_covariance[index] = acceleration_variance
                imu.angular_velocity_covariance[index] = angular_variance
            self._imu_publisher.publish(imu)

            if self._publish_tf:
                transform = TransformStamped()
                transform.header = odometry.header
                transform.child_frame_id = odometry.child_frame_id
                transform.transform.translation.x = state.x
                transform.transform.translation.y = state.y
                transform.transform.rotation = odometry.pose.pose.orientation
                self._tf_broadcaster.sendTransform(transform)

        def _publish_scenario(
            self, friction_coefficient: float, episode_id: str
        ) -> None:
            friction = Float64()
            friction.data = friction_coefficient
            self._friction_publisher.publish(friction)
            episode = String()
            episode.data = episode_id
            self._episode_publisher.publish(episode)

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
