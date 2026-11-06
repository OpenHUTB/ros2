"""ROS 2 simulation node for expert control and state publication."""

from pathlib import Path
from typing import Optional

from .controller import AntagonisticPDController, ControlState
from .simulation import ElbowSimulation


class ElbowSimulationNode:
    """Implementation separated from import-time ROS 2 dependencies."""

    def __init__(
        self, node, joint_state_type, float_type, array_type, package_share: Path
    ) -> None:
        self.node = node
        self.joint_state_type = joint_state_type
        self.float_type = float_type
        self.array_type = array_type
        defaults = {
            "model_file": "models/elbow.xml",
            "publish_rate_hz": 100.0,
            "forearm_load_kg": 0.0,
            "target_angle_rad": 0.8,
            "kp": 12.0,
            "kd": 0.5,
            "torque_scale": 2.0,
            "coactivation": 0.03,
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)
        model_value = str(node.get_parameter("model_file").value)
        model_path = Path(model_value)
        if not model_path.is_absolute():
            model_path = package_share / model_path
        self.target = float(node.get_parameter("target_angle_rad").value)
        self.simulation = ElbowSimulation(
            model_path, float(node.get_parameter("forearm_load_kg").value)
        )
        self.controller = AntagonisticPDController(
            kp=float(node.get_parameter("kp").value),
            kd=float(node.get_parameter("kd").value),
            torque_scale=float(node.get_parameter("torque_scale").value),
            coactivation=float(node.get_parameter("coactivation").value),
        )
        self.joint_publisher = node.create_publisher(
            joint_state_type, "joint_states", 10
        )
        self.force_publisher = node.create_publisher(array_type, "tendon_forces", 10)
        self.activation_publisher = node.create_publisher(
            array_type, "muscle_activations", 10
        )
        self.target_subscription = node.create_subscription(
            float_type, "target_angle", self._target_callback, 10
        )
        rate = float(node.get_parameter("publish_rate_hz").value)
        if rate <= 0.0 or rate > 1000.0:
            raise ValueError("publish_rate_hz must be within (0, 1000]")
        self.timer = node.create_timer(1.0 / rate, self._tick)

    def _target_callback(self, message) -> None:
        target = float(message.data)
        if 0.05 <= target <= 2.20:
            self.target = target
        else:
            self.node.get_logger().warning("Target rejected / 目标角度超出安全范围")

    def _tick(self) -> None:
        try:
            observation = self.simulation.observe()
            command = self.controller.command(
                ControlState(
                    self.target,
                    observation.angle_rad,
                    observation.angular_velocity_rad_s,
                    observation.flexor_force_n,
                    observation.extensor_force_n,
                )
            )
            observation = self.simulation.step(command)
            joint_message = self.joint_state_type()
            joint_message.header.stamp = self.node.get_clock().now().to_msg()
            joint_message.name = ["elbow_joint"]
            joint_message.position = [observation.angle_rad]
            joint_message.velocity = [observation.angular_velocity_rad_s]
            force_message = self.array_type()
            force_message.data = [
                observation.flexor_force_n,
                observation.extensor_force_n,
            ]
            activation_message = self.array_type()
            activation_message.data = list(command)
            self.joint_publisher.publish(joint_message)
            self.force_publisher.publish(force_message)
            self.activation_publisher.publish(activation_message)
        except (
            Exception
        ) as error:  # ROS timer boundary must always force a safe command.
            self.simulation.safe_stop()
            self.node.get_logger().error(f"Safe stop / 安全停机: {error}")
            self.timer.cancel()

    def shutdown(self) -> None:
        self.simulation.safe_stop()


def main(args: Optional[list[str]] = None) -> None:
    try:
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64, Float64MultiArray
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Humble Python packages are required / 需要 ROS 2 Humble"
        ) from error
    rclpy.init(args=args)
    node = Node("musculoskeletal_elbow_simulation")
    implementation = None
    try:
        share = Path(
            get_package_share_directory("mujoco_musculoskeletal_elbow_nn_control")
        )
        implementation = ElbowSimulationNode(
            node, JointState, Float64, Float64MultiArray, share
        )
        rclpy.spin(node)
    finally:
        if implementation is not None:
            implementation.shutdown()
        node.destroy_node()
        rclpy.shutdown()
