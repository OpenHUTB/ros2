"""ROS 2 proprioception node for the two-joint musculoskeletal arm."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .model import (
    DEFAULT_MAX_FORCES_N,
    DEFAULT_MOMENT_ARMS,
    DEFAULT_REST_LENGTHS_M,
    DEFAULT_TENDON_STIFFNESS_N_M,
    MUSCLE_NAMES,
    ProprioceptionModel,
)
from .sensor import NoisyDelayedProprioception


class ArmProprioceptionNode:
    """Combine arm state into noisy and delayed proprioceptive topics."""

    def __init__(
        self,
        node,
        joint_state_type,
        array_type,
    ) -> None:
        self.node = node
        self.joint_state_type = joint_state_type
        self.array_type = array_type

        defaults = {
            'sample_rate_hz': 100.0,
            'delay_s': 0.03,
            'length_noise_std_m': 0.0005,
            'velocity_noise_std_m_s': 0.005,
            'force_noise_std_n': 0.2,
            'tendon_extension_noise_std_m': 0.0001,
            'random_seed': 0,
            'velocity_filter_tau_s': 0.02,
            'muscle_names': list(MUSCLE_NAMES),
            'moment_arms_flat': DEFAULT_MOMENT_ARMS.ravel().tolist(),
            'resting_lengths_m': DEFAULT_REST_LENGTHS_M.tolist(),
            'tendon_stiffness_n_m': (
                DEFAULT_TENDON_STIFFNESS_N_M.tolist()
            ),
            'max_forces_n': DEFAULT_MAX_FORCES_N.tolist(),
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)

        sample_rate_hz = float(node.get_parameter('sample_rate_hz').value)
        if sample_rate_hz <= 0.0 or sample_rate_hz > 1000.0:
            raise ValueError('sample_rate_hz must be within (0, 1000]')
        self.sample_period_s = 1.0 / sample_rate_hz
        muscle_names = tuple(
            str(name) for name in node.get_parameter('muscle_names').value
        )
        moment_arms_flat = np.asarray(
            node.get_parameter('moment_arms_flat').value,
            dtype=float,
        )
        if moment_arms_flat.shape != (len(MUSCLE_NAMES) * 2,):
            raise ValueError(
                'moment_arms_flat must contain 12 coefficients'
            )
        moment_arms = moment_arms_flat.reshape(len(MUSCLE_NAMES), 2)
        self.model = ProprioceptionModel(
            muscle_names=muscle_names,
            moment_arms=moment_arms,
            resting_lengths_m=np.asarray(
                node.get_parameter('resting_lengths_m').value,
                dtype=float,
            ),
            tendon_stiffness_n_m=np.asarray(
                node.get_parameter('tendon_stiffness_n_m').value,
                dtype=float,
            ),
            max_forces_n=np.asarray(
                node.get_parameter('max_forces_n').value,
                dtype=float,
            ),
            velocity_filter_tau_s=float(
                node.get_parameter('velocity_filter_tau_s').value
            ),
        )
        self.sensor = NoisyDelayedProprioception(
            sample_period_s=self.sample_period_s,
            delay_s=float(node.get_parameter('delay_s').value),
            length_noise_std_m=float(
                node.get_parameter('length_noise_std_m').value
            ),
            velocity_noise_std_m_s=float(
                node.get_parameter('velocity_noise_std_m_s').value
            ),
            force_noise_std_n=float(
                node.get_parameter('force_noise_std_n').value
            ),
            tendon_extension_noise_std_m=float(
                node.get_parameter('tendon_extension_noise_std_m').value
            ),
            seed=int(node.get_parameter('random_seed').value),
        )

        self._joint_position = np.zeros(2)
        self._joint_velocity = np.zeros(2)
        self._muscle_forces = np.zeros(len(MUSCLE_NAMES))
        self._activations = np.zeros(len(MUSCLE_NAMES))
        self._received_joint_state = False
        self._received_forces = False
        self._received_activations = False

        self.spindle_length_publisher = node.create_publisher(
            array_type,
            'proprioception/spindle_length',
            10,
        )
        self.spindle_velocity_publisher = node.create_publisher(
            array_type,
            'proprioception/spindle_velocity',
            10,
        )
        self.golgi_force_publisher = node.create_publisher(
            array_type,
            'proprioception/golgi_force',
            10,
        )
        self.tendon_extension_publisher = node.create_publisher(
            array_type,
            'proprioception/tendon_extension',
            10,
        )
        self.joint_subscription = node.create_subscription(
            joint_state_type,
            'joint_states',
            self._joint_state_callback,
            10,
        )
        self.force_subscription = node.create_subscription(
            array_type,
            'muscle_forces',
            self._force_callback,
            10,
        )
        self.activation_subscription = node.create_subscription(
            array_type,
            'muscle_activations',
            self._activation_callback,
            10,
        )
        self.timer = node.create_timer(self.sample_period_s, self._tick)

        node.get_logger().info(
            'Proprioception sensor: %.1f Hz, delay %.1f ms, noise enabled'
            % (sample_rate_hz, 1000.0 * self.sensor.delay_s)
        )
        node.get_logger().info(
            'Waiting for /joint_states, /muscle_forces, /muscle_activations'
        )

    @property
    def inputs_ready(self) -> bool:
        return (
            self._received_joint_state
            and self._received_forces
            and self._received_activations
        )

    def _joint_state_callback(self, message) -> None:
        try:
            positions = {}
            velocities = {}
            velocity_values = message.velocity
            if len(velocity_values) != len(message.name):
                velocity_values = [0.0] * len(message.name)
            for name, position, velocity in zip(
                message.name,
                message.position,
                velocity_values,
            ):
                positions[name] = float(position)
                velocities[name] = float(velocity)
            required = ('shoulder_joint', 'elbow_joint')
            if not all(name in positions for name in required):
                raise ValueError(
                    'missing shoulder_joint or elbow_joint'
                )
            values = np.array(
                [
                    positions['shoulder_joint'],
                    positions['elbow_joint'],
                    velocities['shoulder_joint'],
                    velocities['elbow_joint'],
                ]
            )
            if not np.all(np.isfinite(values)):
                raise ValueError('contains non-finite values')
        except Exception as error:
            self._received_joint_state = False
            self.node.get_logger().warning(
                'joint_states rejected: %s' % error
            )
            return
        self._joint_position = values[:2].copy()
        self._joint_velocity = values[2:].copy()
        self._received_joint_state = True

    def _force_callback(self, message) -> None:
        try:
            values = self._validated_muscle_array(message)
        except Exception as error:
            self._received_forces = False
            self.node.get_logger().warning(
                'muscle_forces rejected: %s' % error
            )
            return
        self._muscle_forces = np.maximum(values, 0.0)
        self._received_forces = True

    def _activation_callback(self, message) -> None:
        try:
            values = self._validated_muscle_array(message)
        except Exception as error:
            self._received_activations = False
            self.node.get_logger().warning(
                'muscle_activations rejected: %s' % error
            )
            return
        self._activations = np.clip(values, 0.0, 1.0)
        self._received_activations = True

    def _validated_muscle_array(self, message):
        try:
            values = np.asarray(message.data, dtype=float)
        except (TypeError, ValueError) as error:
            raise ValueError('values must be numeric') from error
        if values.shape != (len(MUSCLE_NAMES),):
            raise ValueError(
                'must contain %d values' % len(MUSCLE_NAMES)
            )
        if not np.all(np.isfinite(values)):
            raise ValueError('contains non-finite values')
        return values

    def _tick(self) -> None:
        if not self.inputs_ready:
            return
        try:
            sample = self.model.estimate(
                shoulder_rad=float(self._joint_position[0]),
                elbow_rad=float(self._joint_position[1]),
                shoulder_velocity_rad_s=float(self._joint_velocity[0]),
                elbow_velocity_rad_s=float(self._joint_velocity[1]),
                activations=self._activations,
                muscle_forces_n=self._muscle_forces,
                dt_s=self.sample_period_s,
            )
            sensed = self.sensor.measure(sample)
        except Exception as error:
            self.node.get_logger().warning(
                'Proprioception update rejected: %s' % error
            )
            return
        if not self.sensor.ready:
            return

        self._publish(self.spindle_length_publisher, sensed.spindle_length_m)
        self._publish(
            self.spindle_velocity_publisher,
            sensed.spindle_velocity_m_s,
        )
        self._publish(self.golgi_force_publisher, sensed.golgi_force_n)
        self._publish(
            self.tendon_extension_publisher,
            sensed.tendon_extension_m,
        )

    def _publish(self, publisher, values: np.ndarray) -> None:
        message = self.array_type()
        message.data = [float(value) for value in values]
        publisher.publish(message)

    def reset(self) -> None:
        self.model.reset()
        self.sensor.reset()


def main(args: Optional[list[str]] = None) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
    except ImportError as error:
        raise RuntimeError('ROS 2 Humble Python packages are required') from error

    rclpy.init(args=args)
    node = Node('mujoco_msk_arm_proprioception')
    implementation = None
    try:
        implementation = ArmProprioceptionNode(
            node,
            JointState,
            Float64MultiArray,
        )
        rclpy.spin(node)
    finally:
        if implementation is not None:
            implementation.reset()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
