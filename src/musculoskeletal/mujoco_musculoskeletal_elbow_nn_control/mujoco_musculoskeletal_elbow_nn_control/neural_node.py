"""ROS 2 neural controller with deterministic input validation and safe stop."""

import time
from pathlib import Path

import numpy as np

from .mlp import ActivationMLP


class NeuralControllerCore:
    def __init__(self, model_path: Path, timeout_s=0.25):
        self.model = ActivationMLP.load(model_path)
        self.timeout_s = float(timeout_s)
        self.target = None; self.joint = None; self.forces = None; self.updated = 0.0

    def update_target(self, value): self.target = float(value); self.updated = time.monotonic()
    def update_joint(self, angle, velocity): self.joint = (float(angle), float(velocity)); self.updated = time.monotonic()
    def update_forces(self, values):
        values = tuple(float(v) for v in values)
        if len(values) != 2: raise ValueError("two tendon forces are required")
        self.forces = values; self.updated = time.monotonic()

    def predict(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self.target is None or self.joint is None or self.forces is None or now - self.updated > self.timeout_s:
            return np.zeros(2, dtype=float)
        features = [self.target, self.joint[0], self.joint[1], self.forces[0], self.forces[1]]
        return np.clip(self.model.predict(features)[0], 0.0, 1.0)


def main(args=None):
    try:
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64, Float64MultiArray
    except ImportError as error:
        raise RuntimeError("ROS 2 Humble Python packages are required / 需要 ROS 2 Humble") from error
    rclpy.init(args=args)
    node = Node("mujoco_elbow_neural_controller")
    node.declare_parameter("model_file", "models/elbow_mlp.json")
    node.declare_parameter("inference_rate_hz", 100.0)
    node.declare_parameter("input_timeout_s", 0.25)
    model = Path(str(node.get_parameter("model_file").value));
    if not model.is_absolute(): model = Path(get_package_share_directory("mujoco_musculoskeletal_elbow_nn_control")) / model
    core = NeuralControllerCore(model, node.get_parameter("input_timeout_s").value)
    pub = node.create_publisher(Float64MultiArray, "muscle_activations", 10)
    def target_callback(msg): core.update_target(msg.data)
    def joint_callback(msg):
        if msg.position and msg.velocity: core.update_joint(msg.position[0], msg.velocity[0])
    def force_callback(msg): core.update_forces(msg.data)
    node.create_subscription(Float64, "target_angle", target_callback, 10)
    node.create_subscription(JointState, "joint_states", joint_callback, 10)
    node.create_subscription(Float64MultiArray, "tendon_forces", force_callback, 10)
    rate = float(node.get_parameter("inference_rate_hz").value)
    if rate <= 0 or rate > 1000: raise ValueError("inference_rate_hz must be in (0, 1000]")
    def tick():
        msg = Float64MultiArray(); msg.data = core.predict().tolist(); pub.publish(msg)
    node.create_timer(1.0 / rate, tick)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
