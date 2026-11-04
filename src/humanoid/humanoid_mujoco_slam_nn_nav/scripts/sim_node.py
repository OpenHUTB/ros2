#!/usr/bin/env python3
"""仿真节点：MuJoCo 物理 + 双足步态 + 传感器广播。

订阅：
    /cmd_vel            geometry_msgs/Twist      高层速度指令 (linear.x, angular.z)
    /humanoid/reset     std_msgs/Empty           重置到起点

发布：
    /joint_states       sensor_msgs/JointState
    /scan               sensor_msgs/LaserScan
    /camera/depth/image_raw  sensor_msgs/Image     32FC1，米制深度
    /humanoid/odom      nav_msgs/Odometry        里程计（含噪声/漂移）
    /humanoid/ground_truth  geometry_msgs/PoseStamped  仿真真值（仅评测用）
    /humanoid/imu       sensor_msgs/Imu
    /tf                 world -> base_link
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import Empty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from humanoid_mujoco_slam_nn_nav.gait import GaitParams, Walker          # noqa: E402
from humanoid_mujoco_slam_nn_nav.model import ACTUATED_JOINTS            # noqa: E402
from humanoid_mujoco_slam_nn_nav.sensors import DepthCamera, IMU, Lidar, Odometry  # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim                  # noqa: E402


class SimNode:
    def __init__(self):
        self.scene = rospy.get_param("~scene", "warehouse")
        self.seed = int(rospy.get_param("~seed", 0))
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.visualize = bool(rospy.get_param("~visualize", False))

        self.sim = HumanoidSim(scene=self.scene, seed=self.seed, render=self.visualize)
        self.lidar = Lidar(self.sim.model, n_beams=int(rospy.get_param("~lidar_beams", 180)))
        self.depth = DepthCamera(self.sim.model, 32, 24)
        self.imu = IMU()
        self.odom = Odometry()
        sx, sy, syaw = self.sim.layout.start
        self.odom.reset(sx, sy, syaw)
        self.walker = Walker(self._gait_params(), sx, sy, syaw)
        self.cmd = (0.0, 0.0)
        self.last_cmd_t = rospy.Time.now()
        self.last_scan_t = rospy.Time(0)
        self.last_depth_t = rospy.Time(0)
        self.last_odom_t = rospy.Time(0)

        self.pub_joint = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.pub_scan = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.pub_depth = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=1)
        self.pub_odom = rospy.Publisher("/humanoid/odom", Odometry, queue_size=1)
        self.pub_gt = rospy.Publisher("/humanoid/ground_truth", PoseStamped, queue_size=1)
        self.pub_imu = rospy.Publisher("/humanoid/imu", Imu, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/cmd_vel", Twist, self._on_cmd, queue_size=1)
        rospy.Subscriber("/humanoid/reset", Empty, self._on_reset, queue_size=1)

    def _gait_params(self) -> GaitParams:
        return GaitParams(
            pelvis_mode="feet",
            pelvis_height=float(rospy.get_param("~pelvis_height", 0.70)),
            duty=float(rospy.get_param("~duty", 0.78)),
            period=float(rospy.get_param("~period", 1.15)),
            side_shift=float(rospy.get_param("~side_shift", 0.02)),
            foot_spread=float(rospy.get_param("~foot_spread", 0.07)),
            step_height=float(rospy.get_param("~step_height", 0.08)),
            capture_gain=float(rospy.get_param("~capture_gain", 0.35)),
            capture_max=0.08, com_kp=1.0, com_kd=0.1, com_max=0.04,
            roll_gain=0.0, pitch_gain=0.0, ankle_kp=0.0, ankle_kd=0.0)

    # ------------------------------------------------------------------
    def _on_cmd(self, msg: Twist):
        self.cmd = (float(msg.linear.x), float(msg.angular.z))
        self.last_cmd_t = rospy.Time.now()

    def _on_reset(self, _msg):
        self.walker.reset(*self.sim.layout.start)
        self.sim.reset()
        sx, sy, syaw = self.sim.layout.start
        self.odom.reset(sx, sy, syaw)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_cmd_t).to_sec() > 0.5:
                self.cmd = (0.0, 0.0)      # 指令超时保护
            st = self.sim.state()
            self.walker.command(*self.cmd)
            feet = {"l": tuple(self.sim.ankle_pos("l")[:2]),
                    "r": tuple(self.sim.ankle_pos("r")[:2])}
            q = self.walker.update(dt, actual_pos=st["pos"], actual_yaw=st["yaw"],
                                   roll=st["roll"], pitch=st["pitch"], vel=st["vel"],
                                   feet_xy=feet)
            self.sim.set_targets(q)
            self.sim.step(max(1, int(round(dt / self.sim.model.opt.timestep))))

            self._publish_state(now, st)
            if (now - self.last_scan_t).to_sec() >= 0.1:
                self.last_scan_t = now
                self._publish_scan(now)
            if (now - self.last_depth_t).to_sec() >= 0.2:
                self.last_depth_t = now
                self._publish_depth(now)
            rate.sleep()

    def _publish_state(self, now, st):
        js = JointState()
        js.header.stamp = now
        js.name = list(ACTUATED_JOINTS)
        js.position = [st["joints"].get(n, 0.0) for n in js.name]
        self.pub_joint.publish(js)

        gt = PoseStamped()
        gt.header.stamp = now
        gt.header.frame_id = "world"
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = st["pos"]
        quat = tf.transformations.quaternion_from_euler(st["roll"], st["pitch"], st["yaw"])
        gt.pose.orientation.x, gt.pose.orientation.y = quat[0], quat[1]
        gt.pose.orientation.z, gt.pose.orientation.w = quat[2], quat[3]
        self.pub_gt.publish(gt)
        self.tf_broadcaster.sendTransform(
            (st["pos"][0], st["pos"][1], st["pos"][2]), quat, now, "base_link", "world")

        if (now - self.last_odom_t).to_sec() >= 0.1:
            self.last_odom_t = now
            o = self.odom.update(st["vel"][0], st["vel"][1], 0.0, 0.1)
            od = Odometry()
            od.header.stamp = now
            od.header.frame_id = "world"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = o["x"]
            od.pose.pose.position.y = o["y"]
            q2 = tf.transformations.quaternion_from_euler(0, 0, o["yaw"])
            od.pose.pose.orientation.x, od.pose.pose.orientation.y = q2[0], q2[1]
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = q2[2], q2[3]
            od.twist.twist.linear.x = st["vel"][0]
            od.twist.twist.linear.y = st["vel"][1]
            self.pub_odom.publish(od)

            im = Imu()
            im.header.stamp = now
            im.header.frame_id = "base_link"
            r = self.imu.read(self.sim.base_quat(), st["vel"])
            q3 = tf.transformations.quaternion_from_euler(r["roll"], r["pitch"], r["yaw"])
            im.orientation.x, im.orientation.y = q3[0], q3[1]
            im.orientation.z, im.orientation.w = q3[2], q3[3]
            im.angular_velocity.x, im.angular_velocity.y, im.angular_velocity.z = r["gyro"]
            self.pub_imu.publish(im)

    def _publish_scan(self, now):
        ranges = self.lidar.scan(self.sim.data)
        msg = LaserScan()
        msg.header.stamp = now
        msg.header.frame_id = "base_link"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / len(ranges)
        msg.range_min = 0.05
        msg.range_max = 8.0
        msg.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(msg)

    def _publish_depth(self, now):
        depth = self.depth.depth(self.sim.data).astype(np.float32)
        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = "head_cam"
        msg.height, msg.width = depth.shape
        msg.encoding = "32FC1"
        msg.step = depth.shape[1] * 4
        msg.data = depth.tobytes()
        self.pub_depth.publish(msg)


if __name__ == "__main__":
    rospy.init_node("humanoid_sim")
    SimNode().spin()
