"""ROS 2 fatigue dynamics and learning-compensation node."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from mujoco_musculoskeletal_arm_reach.controller import (
    ArmControlState,
    MuscleArmController,
)
from mujoco_musculoskeletal_arm_reach.kinematics import inverse_kinematics
from mujoco_musculoskeletal_arm_reach.simulation import ArmSimulation

from .fatigue import MUSCLE_NAMES, MuscleFatigueModel
from .learning import AdaptiveFatigueCompensator
from .policy import LearningPolicy


def _parameter_vector(node, name: str, size: int) -> np.ndarray:
    values = np.asarray(node.get_parameter(name).value, dtype=float)
    if values.shape != (size,):
        raise ValueError('%s must contain %d values' % (name, size))
    if not np.all(np.isfinite(values)):
        raise ValueError('%s values must be finite' % name)
    return values


class FatigueLearningNode:
    """Run the arm while accumulating fatigue and learning compensation."""

    def __init__(
        self,
        node,
        joint_state_type,
        point_type,
        array_type,
        float_type,
        model_path: Path,
    ) -> None:
        self.node = node
        self.joint_state_type = joint_state_type
        self.point_type = point_type
        self.array_type = array_type
        self.float_type = float_type

        defaults = {
            'publish_rate_hz': 100.0,
            'physics_steps_per_tick': 5,
            'payload_kg': 1.0,
            'target_x_m': 0.38,
            'target_z_m': -0.17,
            'shoulder_kp': 32.0,
            'shoulder_kd': 2.0,
            'elbow_kp': 17.0,
            'elbow_kd': 1.6,
            'coactivation': 0.025,
            'learning_enabled': True,
            'policy_file': '',
            'fatigue_time_s': [7.0, 7.0, 6.0, 6.0, 8.0, 8.0],
            'recovery_time_s': [12.0, 12.0, 10.0, 10.0, 14.0, 14.0],
            'strength_loss': [0.32, 0.32, 0.36, 0.36, 0.30, 0.30],
            'activation_threshold': 0.05,
            'maximum_fatigue': 0.95,
            'learning_rate': 100.0,
            'gain_decay_rate': 0.20,
            'maximum_gain': 2.0,
            'activation_deadband': 0.02,
            'error_deadband_rad': 0.001,
            'error_ceiling_rad': 0.30,
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)

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
        self.fatigue_model = MuscleFatigueModel(
            fatigue_time_s=_parameter_vector(
                node,
                'fatigue_time_s',
                len(MUSCLE_NAMES),
            ),
            recovery_time_s=_parameter_vector(
                node,
                'recovery_time_s',
                len(MUSCLE_NAMES),
            ),
            strength_loss=_parameter_vector(
                node,
                'strength_loss',
                len(MUSCLE_NAMES),
            ),
            activation_threshold=float(
                node.get_parameter('activation_threshold').value
            ),
            maximum_fatigue=float(
                node.get_parameter('maximum_fatigue').value
            ),
        )

        policy = LearningPolicy(
            learning_rate=float(
                node.get_parameter('learning_rate').value
            ),
            gain_decay_rate=float(
                node.get_parameter('gain_decay_rate').value
            ),
            maximum_gain=float(
                node.get_parameter('maximum_gain').value
            ),
            activation_deadband=float(
                node.get_parameter('activation_deadband').value
            ),
            error_deadband_rad=float(
                node.get_parameter('error_deadband_rad').value
            ),
            error_ceiling_rad=float(
                node.get_parameter('error_ceiling_rad').value
            ),
        )
        policy_path = str(node.get_parameter('policy_file').value).strip()
        if policy_path:
            expanded_path = Path(policy_path).expanduser()
            if expanded_path.exists():
                policy = LearningPolicy.load(expanded_path)
                node.get_logger().info(
                    'Loaded learning policy: %s' % expanded_path
                )
            else:
                node.get_logger().warning(
                    'Learning policy not found, using YAML parameters: %s'
                    % expanded_path
                )
        self.learning_enabled = bool(
            node.get_parameter('learning_enabled').value
        )
        self.compensator = AdaptiveFatigueCompensator(
            learning_rate=policy.learning_rate,
            gain_decay_rate=policy.gain_decay_rate,
            maximum_gain=policy.maximum_gain,
            activation_deadband=policy.activation_deadband,
            error_deadband_rad=policy.error_deadband_rad,
            error_ceiling_rad=policy.error_ceiling_rad,
        )

        self.target = (
            float(node.get_parameter('target_x_m').value),
            float(node.get_parameter('target_z_m').value),
        )
        inverse_kinematics(*self.target, self.controller.geometry)

        publish_rate = float(node.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0 or publish_rate > 1000.0:
            raise ValueError('publish_rate_hz must be within (0, 1000]')
        self.sample_period_s = 1.0 / publish_rate
        self.physics_steps_per_tick = int(
            node.get_parameter('physics_steps_per_tick').value
        )
        if not 1 <= self.physics_steps_per_tick <= 100:
            raise ValueError(
                'physics_steps_per_tick must be within [1, 100]'
            )

        self.joint_publisher = node.create_publisher(
            joint_state_type,
            'joint_states',
            10,
        )
        self.tip_publisher = node.create_publisher(
            point_type,
            'end_effector_position',
            10,
        )
        self.force_publisher = node.create_publisher(
            array_type,
            'muscle_forces',
            10,
        )
        self.activation_publisher = node.create_publisher(
            array_type,
            'muscle_activations',
            10,
        )
        self.fatigue_publisher = node.create_publisher(
            array_type,
            'fatigue/state',
            10,
        )
        self.gain_publisher = node.create_publisher(
            array_type,
            'fatigue/compensation_gain',
            10,
        )
        self.error_publisher = node.create_publisher(
            float_type,
            'fatigue/tracking_error',
            10,
        )
        self.target_subscription = node.create_subscription(
            point_type,
            'target_position',
            self._target_callback,
            10,
        )
        self.timer = node.create_timer(self.sample_period_s, self._tick)
        node.get_logger().info(
            'Fatigue learning enabled=%s, payload=%.2f kg, rate=%.1f Hz'
            % (
                self.learning_enabled,
                float(node.get_parameter('payload_kg').value),
                publish_rate,
            )
        )

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
        self.node.get_logger().info(
            'Target updated: x=%.3f m, z=%.3f m' % target
        )

    def _tick(self) -> None:
        try:
            observation = self.simulation.observe()
            base_activation = np.asarray(
                self.controller.command(
                    ArmControlState(
                        target_x_m=self.target[0],
                        target_z_m=self.target[1],
                        shoulder_rad=observation.shoulder_rad,
                        elbow_rad=observation.elbow_rad,
                        shoulder_velocity_rad_s=(
                            observation.shoulder_velocity_rad_s
                        ),
                        elbow_velocity_rad_s=(
                            observation.elbow_velocity_rad_s
                        ),
                    )
                )
            )
            fatigue_state = self.fatigue_model.update(
                base_activation,
                self.sample_period_s,
            )
            target_joint = np.asarray(
                inverse_kinematics(
                    *self.target,
                    self.controller.geometry,
                )
            )
            joint_error = target_joint - np.array(
                [observation.shoulder_rad, observation.elbow_rad]
            )
            if self.learning_enabled:
                gain = self.compensator.update(
                    base_activation,
                    joint_error,
                    self.sample_period_s,
                )
                effective_activation = self.compensator.compensate(
                    base_activation,
                    fatigue_state.strength_retention,
                )
            else:
                gain = np.ones(len(MUSCLE_NAMES))
                effective_activation = np.clip(
                    base_activation
                    * fatigue_state.strength_retention,
                    0.0,
                    1.0,
                )

            for _ in range(self.physics_steps_per_tick):
                observation = self.simulation.step(
                    tuple(effective_activation)
                )
            self._publish_state(
                observation,
                effective_activation,
                fatigue_state,
                gain,
                float(np.linalg.norm(joint_error)),
            )
        except Exception as error:
            self.simulation.safe_stop()
            self.node.get_logger().error(
                'Simulation stopped safely: %s' % error
            )
            self.timer.cancel()

    def _publish_state(
        self,
        observation,
        effective_activation: np.ndarray,
        fatigue_state,
        gain: np.ndarray,
        tracking_error_rad: float,
    ) -> None:
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
        activation_message.data = [
            float(value) for value in effective_activation
        ]
        fatigue_message = self.array_type()
        fatigue_message.data = [
            float(value) for value in fatigue_state.fatigue
        ]
        gain_message = self.array_type()
        gain_message.data = [float(value) for value in gain]
        error_message = self.float_type()
        error_message.data = float(tracking_error_rad)

        self.joint_publisher.publish(joint_message)
        self.tip_publisher.publish(tip_message)
        self.force_publisher.publish(force_message)
        self.activation_publisher.publish(activation_message)
        self.fatigue_publisher.publish(fatigue_message)
        self.gain_publisher.publish(gain_message)
        self.error_publisher.publish(error_message)

    def shutdown(self) -> None:
        self.simulation.safe_stop()


def main(args: Optional[list[str]] = None) -> None:
    try:
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import Point
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32, Float64MultiArray
    except ImportError as error:
        raise RuntimeError('ROS 2 Humble Python packages are required') from error

    rclpy.init(args=args)
    node = Node('mujoco_msk_arm_fatigue_learning')
    implementation = None
    try:
        arm_share = Path(
            get_package_share_directory(
                'mujoco_musculoskeletal_arm_reach'
            )
        )
        implementation = FatigueLearningNode(
            node,
            JointState,
            Point,
            Float64MultiArray,
            Float32,
            arm_share / 'models' / 'arm.xml',
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
