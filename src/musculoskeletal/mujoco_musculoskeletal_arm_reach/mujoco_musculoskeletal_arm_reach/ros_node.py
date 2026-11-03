"""ROS 2 node for the two-joint musculoskeletal arm."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from .controller import ArmControlState, MuscleArmController
from .kinematics import inverse_kinematics
from .simulation import ArmSimulation


class ArmSimulationNode:
    """Timer-driven MuJoCo simulation separated from ROS imports."""

    def __init__(
        self,
        node,
        joint_state_type,
        point_type,
        array_type,
        package_share: Path,
    ) -> None:
        self.node = node
        self.joint_state_type = joint_state_type
        self.point_type = point_type
        self.array_type = array_type

        defaults = {
            'model_file': 'models/arm.xml',
            'publish_rate_hz': 100.0,
            'physics_steps_per_tick': 5,
            'payload_kg': 0.0,
            'target_x_m': 0.38,
            'target_z_m': -0.17,
            'shoulder_kp': 32.0,
            'shoulder_kd': 2.0,
            'elbow_kp': 17.0,
            'elbow_kd': 1.6,
            'coactivation': 0.025,
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)

        model_value = str(node.get_parameter('model_file').value)
        model_path = Path(model_value)
        if not model_path.is_absolute():
            model_path = package_share / model_path
        self.simulation = ArmSimulation(
            model_path,
            payload_kg=float(node.get_parameter('payload_kg').value),
        )
        self.controller = MuscleArmController(
            shoulder_kp=float(node.get_parameter('shoulder_kp').value),
            shoulder_kd=float(node.get_parameter('shoulder_kd').value),
            elbow_kp=float(node.get_parameter('elbow_kp').value),
            elbow_kd=float(node.get_parameter('elbow_kd').value),
            coactivation=float(node.get_parameter('coactivation').value),
        )
        self.target = (
            float(node.get_parameter('target_x_m').value),
            float(node.get_parameter('target_z_m').value),
        )
        inverse_kinematics(*self.target, self.controller.geometry)

        self.joint_publisher = node.create_publisher(
            joint_state_type, 'joint_states', 10
        )
        self.tip_publisher = node.create_publisher(
            point_type, 'end_effector_position', 10
        )
        self.force_publisher = node.create_publisher(
            array_type, 'muscle_forces', 10
        )
        self.activation_publisher = node.create_publisher(
            array_type, 'muscle_activations', 10
        )
        self.target_subscription = node.create_subscription(
            point_type, 'target_position', self._target_callback, 10
        )

        publish_rate = float(node.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0 or publish_rate > 1000.0:
            raise ValueError('publish_rate_hz must be within (0, 1000]')
        self.physics_steps_per_tick = int(
            node.get_parameter('physics_steps_per_tick').value
        )
        if not 1 <= self.physics_steps_per_tick <= 100:
            raise ValueError('physics_steps_per_tick must be within [1, 100]')
        self.timer = node.create_timer(1.0 / publish_rate, self._tick)

    def _target_callback(self, message) -> None:
        target = (float(message.x), float(message.z))
        try:
            inverse_kinematics(*target, self.controller.geometry)
        except ValueError as error:
            self.node.get_logger().warning(
                'Target rejected: %s' % error
            )
            return
        self.target = target

    def _tick(self) -> None:
        try:
            observation = self.simulation.observe()
            command = self.controller.command(
                ArmControlState(
                    target_x_m=self.target[0],
                    target_z_m=self.target[1],
                    shoulder_rad=observation.shoulder_rad,
                    elbow_rad=observation.elbow_rad,
                    shoulder_velocity_rad_s=(
                        observation.shoulder_velocity_rad_s
                    ),
                    elbow_velocity_rad_s=observation.elbow_velocity_rad_s,
                )
            )
            for _ in range(self.physics_steps_per_tick):
                observation = self.simulation.step(command)

            joint_message = self.joint_state_type()
            joint_message.header.stamp = self.node.get_clock().now().to_msg()
            joint_message.name = ['shoulder_joint', 'elbow_joint']
            joint_message.position = [
                observation.shoulder_rad,
                observation.elbow_rad,
            ]
            joint_message.velocity = [
                observation.shoulder_velocity_rad_s,
                observation.elbow_velocity_rad_s,
            ]

            tip_message = self.point_type()
            tip_message.x = observation.end_effector_x_m
            tip_message.y = 0.0
            tip_message.z = observation.end_effector_z_m

            force_message = self.array_type()
            force_message.data = list(observation.muscle_forces_n)
            activation_message = self.array_type()
            activation_message.data = list(command)

            self.joint_publisher.publish(joint_message)
            self.tip_publisher.publish(tip_message)
            self.force_publisher.publish(force_message)
            self.activation_publisher.publish(activation_message)
        except Exception as error:  # timer boundary: fail safe and stop
            self.simulation.safe_stop()
            self.node.get_logger().error(
                'Simulation stopped safely: %s' % error
            )
            self.timer.cancel()

    def shutdown(self) -> None:
        self.simulation.safe_stop()


def main(args: Optional[list[str]] = None) -> None:
    try:
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import Point
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
    except ImportError as error:
        raise RuntimeError('ROS 2 Humble Python packages are required') from error

    rclpy.init(args=args)
    node = Node('mujoco_musculoskeletal_arm_reach')
    implementation = None
    try:
        share = Path(
            get_package_share_directory('mujoco_musculoskeletal_arm_reach')
        )
        implementation = ArmSimulationNode(
            node,
            JointState,
            Point,
            Float64MultiArray,
            share,
        )
        rclpy.spin(node)
    finally:
        if implementation is not None:
            implementation.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
