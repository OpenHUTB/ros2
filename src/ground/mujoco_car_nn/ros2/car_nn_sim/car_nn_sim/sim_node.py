#!/usr/bin/env python3
"""ROS 2 (Humble) simulation node for the MuJoCo neural-network car.

Publishes camera / lidar / odometry, consumes ``/car/cmd_vel`` (or drives with
a neural-network policy), and broadcasts the TF tree
``odom -> base_link -> {camera_link, laser_link}`` so that RViz and
``slam_node`` can transform every sensor frame.
"""

import os
import sys

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _bootstrap_mujoco_car_nn() -> None:
    """Make ``mujoco_car_nn`` importable from a dev tree or an installed workspace.

    The old hard-coded ``../../../src`` only works when the file is read from
    the source tree; after ``colcon build`` (non-symlink install) ``__file__``
    lives under ``install/`` and the relative path no longer exists.  We try,
    in order: ``$CAR_NN_SRC``, the source layout resolved through
    ``realpath`` (works with ``--symlink-install``), the current directory and
    the two usual ``src`` layouts.  If nothing matches we stop with an
    actionable message instead of a bare ``ModuleNotFoundError``.
    """
    import importlib.util

    if importlib.util.find_spec("mujoco_car_nn") is not None:
        return

    here = os.path.dirname(os.path.realpath(__file__))
    candidates = []
    env = os.environ.get("CAR_NN_SRC")
    if env:
        candidates.append(env)
    candidates += [
        os.path.abspath(os.path.join(here, "..", "..", "..", "src")),
        os.path.abspath(os.path.join(os.getcwd(), "src", "ground", "mujoco_car_nn", "src")),
        os.path.abspath(os.path.join(os.getcwd(), "src")),
        os.path.expanduser("~/ros2_ws/src/ground/mujoco_car_nn/src"),
    ]
    for root in candidates:
        if root and os.path.isdir(os.path.join(root, "mujoco_car_nn")):
            if root not in sys.path:
                sys.path.insert(0, root)
            return
    raise SystemExit(
        "找不到 mujoco_car_nn 包：请设置环境变量 CAR_NN_SRC 指向包含 mujoco_car_nn 的目录，例如\n"
        "  export CAR_NN_SRC=<repo>/src/ground/mujoco_car_nn/src\n"
        "或先执行 pip install -e <repo>/src/ground/mujoco_car_nn/src"
    )


_bootstrap_mujoco_car_nn()

from mujoco_car_nn.sim import LIDAR_RANGE, CarSim, SimConfig  # noqa: E402
from mujoco_car_nn.track import build_track  # noqa: E402


def _transform(stamp, parent: str, child: str, xyz, yaw: float = 0.0) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    t.transform.rotation.z = float(np.sin(yaw / 2.0))
    t.transform.rotation.w = float(np.cos(yaw / 2.0))
    return t


class CarSimNode(Node):
    def __init__(self) -> None:
        super().__init__("car_nn_sim")
        self.declare_parameter("scene", "day")
        self.declare_parameter("policy", "")
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("rate", 20.0)

        scene = self.get_parameter("scene").value
        policy_path = self.get_parameter("policy").value
        rate = float(self.get_parameter("rate").value)
        offset = float(self.get_parameter("offset").value)

        track = build_track()
        self.sim = CarSim(track, SimConfig(scene=scene, seed=0, randomize_start=False))
        self.st = self.sim.reset(offset)
        self.policy = None
        if policy_path:
            from mujoco_car_nn.cli import load_policy

            self.policy = load_policy(policy_path)
            self.get_logger().info(f"driving with policy: {policy_path}")

        self.pub_img = self.create_publisher(Image, "/car/camera/image_raw", 1)
        self.pub_scan = self.create_publisher(LaserScan, "/car/scan", 1)
        self.pub_odom = self.create_publisher(Odometry, "/car/odom", 1)
        self.create_subscription(Twist, "/car/cmd_vel", self.on_cmd, 1)

        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)

        stamp = self.get_clock().now().to_msg()
        # sensor mounting points in the base_link frame (metres)
        self.tf_static.sendTransform(
            [
                _transform(stamp, "base_link", "camera_link", (1.45, 0.0, 1.40)),
                _transform(stamp, "base_link", "laser_link", (0.0, 0.0, 0.60)),
            ]
        )

        self.cmd = [0.0, 0.0]
        self.timer = self.create_timer(1.0 / rate, self.step)

    def on_cmd(self, msg: Twist) -> None:
        self.cmd[0] = float(np.clip(msg.angular.z, -1.0, 1.0))
        self.cmd[1] = float(np.clip(msg.linear.x / 8.0, -1.0, 1.0))

    def step(self) -> None:
        stamp = self.get_clock().now().to_msg()
        img = self.sim.camera_image()
        scan = self.sim.lidar()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.header.frame_id = "laser_link"
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = 2 * np.pi / len(scan)
        scan_msg.range_min = 0.05
        scan_msg.range_max = LIDAR_RANGE
        scan_msg.ranges = scan.astype(np.float32).tolist()
        self.pub_scan.publish(scan_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.st.x
        odom.pose.pose.position.y = self.st.y
        odom.pose.pose.orientation.z = float(np.sin(self.st.yaw / 2))
        odom.pose.pose.orientation.w = float(np.cos(self.st.yaw / 2))
        odom.twist.twist.linear.x = self.st.speed
        odom.twist.twist.angular.z = self.st.yaw_rate
        self.pub_odom.publish(odom)

        # odom -> base_link, keeps the sensor frames connected to the map
        self.tf.sendTransform(
            _transform(stamp, "odom", "base_link", (self.st.x, self.st.y, 0.0), self.st.yaw)
        )

        action = self.policy(img, self.st.speed) if self.policy is not None else np.array(self.cmd, dtype=np.float32)
        self.st = self.sim.step(action)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
