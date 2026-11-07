"""ROS 2 collector for synchronized encoder, IMU, command, and truth data."""

import importlib.metadata
import math
from pathlib import Path
import platform
from typing import Any

from .dataset import DatasetSample, EpisodeCsvWriter, assign_episode_split
from .simulator import MujocoSimulation, quaternion_to_yaw


def _message_timestamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def main(args: list[str] | None = None) -> None:
    """Run the episode-aware ROS 2 data collector."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float64, String
    except ImportError as exc:
        raise RuntimeError(
            '缺少 ROS 2 Humble Python 依赖 / ROS 2 Humble Python dependencies are required'
        ) from exc

    class DataCollectorNode(Node):
        def __init__(self) -> None:
            super().__init__('mujoco_slip_data_collector')
            self.declare_parameter('output_directory', 'data/raw')
            self.declare_parameter('split_seed', 25012)
            self.declare_parameter('random_seed', 25012)
            self.declare_parameter('flush_interval', 100)
            self._output_directory = Path(
                str(self.get_parameter('output_directory').value)
            )
            self._split_seed = int(self.get_parameter('split_seed').value)
            self._random_seed = int(self.get_parameter('random_seed').value)
            self._flush_interval = int(self.get_parameter('flush_interval').value)
            self._episode_id: str | None = None
            self._friction: float | None = None
            self._command: Any | None = None
            self._imu: Any | None = None
            self._odometry: Any | None = None
            self._writer: EpisodeCsvWriter | None = None

            self.create_subscription(
                String, 'scenario/episode_id', self._on_episode, 10
            )
            self.create_subscription(
                Float64, 'scenario/friction', self._on_friction, 10
            )
            self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
            self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
            self.create_subscription(
                Odometry, 'ground_truth/odom', self._on_odometry, 10
            )
            self.create_subscription(
                JointState, 'wheel/encoders', self._on_encoders, 10
            )
            self.get_logger().info(
                '数据采集器已启动 / episode-aware data collector started'
            )

        def _on_episode(self, message: Any) -> None:
            episode_id = str(message.data)
            assign_episode_split(episode_id, self._split_seed)
            if episode_id == self._episode_id:
                return
            if self._writer is not None:
                self._writer.close()
            self._episode_id = episode_id
            self._writer = EpisodeCsvWriter(
                self._output_directory,
                episode_id,
                self._split_seed,
                metadata={
                    'split_seed': self._split_seed,
                    'random_seed': self._random_seed,
                    'python_version': platform.python_version(),
                    'mujoco_version': importlib.metadata.version('mujoco'),
                    'package_version': '0.1.0',
                },
                flush_interval=self._flush_interval,
            )
            self.get_logger().info(
                f'开始 episode / started episode: {episode_id}'
            )

        def _on_friction(self, message: Any) -> None:
            value = float(message.data)
            if not math.isfinite(value) or value <= 0.0:
                self.get_logger().error(
                    '拒绝无效摩擦系数 / rejected invalid friction coefficient'
                )
                return
            self._friction = value

        def _on_command(self, message: Any) -> None:
            self._command = message

        def _on_imu(self, message: Any) -> None:
            self._imu = message

        def _on_odometry(self, message: Any) -> None:
            self._odometry = message

        def _on_encoders(self, message: Any) -> None:
            if any(
                value is None
                for value in (
                    self._writer,
                    self._episode_id,
                    self._friction,
                    self._command,
                    self._imu,
                    self._odometry,
                )
            ):
                return
            index_by_name = {name: index for index, name in enumerate(message.name)}
            try:
                indices = [
                    index_by_name[name]
                    for name in MujocoSimulation.WHEEL_JOINT_NAMES
                ]
                position = [message.position[index] for index in indices]
                velocity = [message.velocity[index] for index in indices]
            except (KeyError, IndexError):
                self.get_logger().error(
                    '编码器消息缺少四轮字段 / encoder message lacks four-wheel fields'
                )
                return
            orientation = self._odometry.pose.pose.orientation
            yaw = quaternion_to_yaw(
                (orientation.w, orientation.x, orientation.y, orientation.z)
            )
            sample = DatasetSample(
                timestamp_ns=_message_timestamp_ns(message),
                episode_id=self._episode_id,
                split=self._writer.split,
                friction_coefficient=self._friction,
                command_linear_x=float(self._command.linear.x),
                command_angular_z=float(self._command.angular.z),
                wheel_position_left_front=float(position[0]),
                wheel_position_left_rear=float(position[1]),
                wheel_position_right_front=float(position[2]),
                wheel_position_right_rear=float(position[3]),
                wheel_velocity_left_front=float(velocity[0]),
                wheel_velocity_left_rear=float(velocity[1]),
                wheel_velocity_right_front=float(velocity[2]),
                wheel_velocity_right_rear=float(velocity[3]),
                imu_acceleration_x=float(self._imu.linear_acceleration.x),
                imu_acceleration_y=float(self._imu.linear_acceleration.y),
                imu_acceleration_z=float(self._imu.linear_acceleration.z),
                imu_angular_velocity_x=float(self._imu.angular_velocity.x),
                imu_angular_velocity_y=float(self._imu.angular_velocity.y),
                imu_angular_velocity_z=float(self._imu.angular_velocity.z),
                ground_truth_x=float(self._odometry.pose.pose.position.x),
                ground_truth_y=float(self._odometry.pose.pose.position.y),
                ground_truth_yaw=yaw,
                ground_truth_linear_x=float(self._odometry.twist.twist.linear.x),
                ground_truth_angular_z=float(self._odometry.twist.twist.angular.z),
            )
            self._writer.write(sample)

        def destroy_node(self) -> bool:
            if self._writer is not None:
                self._writer.close()
            return super().destroy_node()

    node: Any | None = None
    rclpy.init(args=args)
    try:
        node = DataCollectorNode()
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
