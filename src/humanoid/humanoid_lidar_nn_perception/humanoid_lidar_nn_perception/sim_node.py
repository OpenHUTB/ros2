"""MuJoCo simulation node: steps the G1 model, publishes /scan, /joint_states, /odom.

Subscribes to /cmd_vel_safe (geometry_msgs/Twist) for (vx, wz) high-level
commands, which it feeds into the pre-trained ONNX walking policy.
"""
from __future__ import annotations
import os
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

from .g1_walking_policy import G1WalkingPolicy, SIM_DT, N_SUBSTEPS
from .lidar_2d import LiDAR2D


class SimNode(Node):
    def __init__(self):
        super().__init__("g1_sim_node")
        share = get_package_share_directory("humanoid_lidar_nn_perception")
        model_path = os.path.join(share, "models", "g1", "scene_g1.xml")
        onnx_path = os.path.join(share, "weights", "g1_policy.onnx")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

        self.policy = G1WalkingPolicy(self.model, onnx_path)
        self.lidar = LiDAR2D(self.model, num_beams=360, max_range=8.0)

        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_cb, 10)

        # 500 Hz physics timer; control at 50 Hz
        self.timer = self.create_timer(SIM_DT, self._step)
        self.ctrl_counter = 0
        self.scan_counter = 0
        self.angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)

        self.get_logger().info(f"SimNode ready. model={model_path}")

    def _cmd_cb(self, msg: Twist):
        self.policy.set_command(msg.linear.x, 0.0, msg.angular.z)

    def _step(self):
        mujoco.mj_step(self.model, self.data)
        self.ctrl_counter += 1
        if self.ctrl_counter % N_SUBSTEPS == 0:
            self.policy.step(self.data)

        # publish scan at 10 Hz
        self.scan_counter += 1
        if self.scan_counter % int(0.1 / SIM_DT) == 0:
            scan = self.lidar.scan(self.data)
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.angle_min = float(self.angles[0])
            msg.angle_max = float(self.angles[-1] + (self.angles[1] - self.angles[0]))
            msg.angle_increment = float(self.angles[1] - self.angles[0])
            msg.range_min = 0.1
            msg.range_max = 8.0
            msg.ranges = scan.tolist()
            self.pub_scan.publish(msg)

        # joint states at 50 Hz
        if self.ctrl_counter % int(0.02 / SIM_DT) == 0:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [f"joint_{i}" for i in range(29)]
            js.position = self.data.qpos[7:36].tolist()
            self.pub_joint.publish(js)

        # odom at 20 Hz
        if self.ctrl_counter % int(0.05 / SIM_DT) == 0:
            od = Odometry()
            od.header.stamp = self.get_clock().now().to_msg()
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = float(self.data.qpos[0])
            od.pose.pose.position.y = float(self.data.qpos[1])
            od.pose.pose.position.z = float(self.data.qpos[2])
            od.pose.pose.orientation.w = float(self.data.qpos[3])
            od.pose.pose.orientation.x = float(self.data.qpos[4])
            od.pose.pose.orientation.y = float(self.data.qpos[5])
            od.pose.pose.orientation.z = float(self.data.qpos[6])
            self.pub_odom.publish(od)


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
