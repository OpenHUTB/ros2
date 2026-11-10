#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双足机器人仿真节点。

关节由 PD 伺服驱动（等效惯量 + 阻尼），基座按控制器给定的浮动基座状态运动。
节点计算机械臂式正向运动学、质心与 ZMP，并发布给控制/记录节点。
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

from biped_cubic_spline_gait_switch.config_loader import (
    build_model, default_params_path, load_config)
from biped_cubic_spline_gait_switch.kinematics import BipedModel


class SimNode(Node):
    def __init__(self):
        super().__init__("biped_sim")
        self.declare_parameter("config", default_params_path())
        cfg_path = self.get_parameter("config").value
        cfg = load_config(cfg_path)
        self.model: BipedModel = build_model(cfg)

        servo = cfg["servo"]
        self.kp = float(servo["kp"])
        self.kd = float(servo["kd"])
        self.inertia = float(servo["inertia"])
        self.gravity = float(servo.get("gravity", 9.81))
        self.q_min = np.array([
            cfg["joint_limits"]["hip"][0], cfg["joint_limits"]["knee"][0],
            cfg["joint_limits"]["ankle"][0],
            cfg["joint_limits"]["hip"][0], cfg["joint_limits"]["knee"][0],
            cfg["joint_limits"]["ankle"][0],
        ])
        self.q_max = np.array([
            cfg["joint_limits"]["hip"][1], cfg["joint_limits"]["knee"][1],
            cfg["joint_limits"]["ankle"][1],
            cfg["joint_limits"]["hip"][1], cfg["joint_limits"]["knee"][1],
            cfg["joint_limits"]["ankle"][1],
        ])

        self.base_x = 0.0
        self.base_v = 0.0
        self.hip_z = 0.84
        self.lean = 0.0
        self.q_des = np.zeros(6)
        self.qd_des = np.zeros(6)
        init_left = self.model.leg_ik((0.0, self.hip_z), (0.10, self.model.ankle_height))
        init_right = self.model.leg_ik((0.0, self.hip_z), (-0.10, self.model.ankle_height))
        self.q = np.concatenate([init_left, init_right])
        self.q_des = self.q.copy()
        self.qd = np.zeros(6)
        # 接触状态由控制器按规划相位给出（/gait_info），不再用踝高阈值判定
        self.contacts_plan = [True, True]

        com_init = self.model.center_of_mass(0.0, self.hip_z, 0.0, self.q)
        self.com_x = float(com_init[0])
        self.com_z = float(com_init[1])
        self.com_hist = deque(maxlen=200)

        self.create_subscription(JointState, "/joint_cmd", self.on_joint_cmd, 10)
        self.create_subscription(Float32MultiArray, "/base_cmd", self.on_base_cmd, 10)
        self.create_subscription(String, "/gait_info", self.on_gait_info, 20)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.base_pub = self.create_publisher(Odometry, "/base_state", 10)
        self.com_pub = self.create_publisher(PointStamped, "/com", 10)
        self.zmp_pub = self.create_publisher(PointStamped, "/zmp", 10)
        self.metric_pub = self.create_publisher(Float32MultiArray, "/biped_metrics", 10)

        rate = float(servo["rate"])
        self.dt = 1.0 / rate
        self.last_step_time = time.time()
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            "双足仿真节点启动：PD kp=%.0f kd=%.0f，步进 %.0f Hz" % (self.kp, self.kd, rate))

    def on_joint_cmd(self, msg: JointState):
        if len(msg.position) >= 6:
            self.q_des = np.array(msg.position[:6], dtype=float)
        if len(msg.velocity) >= 6:
            self.qd_des = np.array(msg.velocity[:6], dtype=float)

    def on_base_cmd(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) >= 2:
            self.base_x = float(data[0])
            self.base_v = float(data[1])
        if len(data) >= 3:
            self.hip_z = float(data[2])
        if len(data) >= 4:
            self.lean = float(data[3])

    def on_gait_info(self, msg: String):
        try:
            info = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        contacts = info.get("contacts")
        if isinstance(contacts, (list, tuple)) and len(contacts) >= 2:
            self.contacts_plan = [bool(contacts[0]), bool(contacts[1])]

    def step(self):
        now = time.time()
        dt = max(0.0005, min(0.02, now - self.last_step_time))
        self.last_step_time = now

        # 用固定小步长子步进，保证定时器抖动时仍按真实时间推进且数值稳定
        n_sub = max(1, int(round(dt / self.dt)))
        sub_dt = dt / n_sub
        for _ in range(n_sub):
            qdd = (self.kp * (self.q_des - self.q)
                   + self.kd * (self.qd_des - self.qd)) / self.inertia
            self.qd += qdd * sub_dt
            self.q += self.qd * sub_dt
            out_low = self.q < self.q_min
            out_high = self.q > self.q_max
            if np.any(out_low) or np.any(out_high):
                self.q = np.clip(self.q, self.q_min, self.q_max)
                self.qd[out_low | out_high] = 0.0

        # 接触与脚踏位置都按规划量（q_des 的 FK）计算，保证与支撑多边形同源
        contacts = list(self.contacts_plan)
        feet = []
        for idx in range(2):
            fk = self.model.leg_fk((self.base_x, self.hip_z),
                                   self.q_des[3 * idx:3 * idx + 3])
            ankle = fk["ankle"]
            feet.append((float(ankle[0]), float(ankle[1])))

        com = self.model.center_of_mass(self.base_x, self.hip_z, self.lean, self.q)
        self.com_x = float(com[0])
        self.com_z = float(com[1])

        # 用期望关节轨迹的质心计算 ZMP，并用 0.16 s 跨度的二阶差分抑制伺服噪声
        com_des = self.model.center_of_mass(self.base_x, self.hip_z, self.lean, self.q_des)
        self.com_hist.append((now, float(com_des[0]), float(com_des[1])))
        zmp_x = self.com_x
        if len(self.com_hist) >= 80:
            t_now, x_now, z_now = self.com_hist[-1]
            t_mid, x_mid, _ = self.com_hist[-40]
            t_old, x_old, _ = self.com_hist[-80]
            dt1 = max(t_now - t_mid, 1e-4)
            dt0 = max(t_mid - t_old, 1e-4)
            v1 = (x_now - x_mid) / dt1
            v0 = (x_mid - x_old) / dt0
            ax = (v1 - v0) / (0.5 * (dt1 + dt0))
            zmp_x = float(com_des[0]) - float(com_des[1]) * ax / self.gravity

        support_min, support_max = self.model.support_polygon(
            self.base_x, self.hip_z, self.lean, self.q_des, contacts)
        if support_min is None:
            margin = float("nan")
        else:
            margin = min(zmp_x - support_min, support_max - zmp_x)

        self.publish_all(feet, contacts, zmp_x, margin)

    def publish_all(self, feet, contacts, zmp_x, margin):
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = ["left_hip", "left_knee", "left_ankle",
                   "right_hip", "right_knee", "right_ankle"]
        js.position = [float(v) for v in self.q]
        js.velocity = [float(v) for v in self.qd]
        self.joint_pub.publish(js)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.base_x
        odom.pose.pose.position.z = self.hip_z
        odom.pose.pose.orientation.y = math.sin(self.lean / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.lean / 2.0)
        odom.twist.twist.linear.x = self.base_v
        self.base_pub.publish(odom)

        com_msg = PointStamped()
        com_msg.header = odom.header
        com_msg.point.x = self.com_x
        com_msg.point.y = 0.0
        com_msg.point.z = self.com_z
        self.com_pub.publish(com_msg)

        zmp_msg = PointStamped()
        zmp_msg.header = odom.header
        zmp_msg.point.x = zmp_x
        zmp_msg.point.z = 0.0
        zmp_msg.point.y = 0.0
        self.zmp_pub.publish(zmp_msg)

        metrics = Float32MultiArray()
        metrics.data = [
            float(self.com_x), float(self.com_z), float(zmp_x), float(margin),
            float(feet[0][0]), float(feet[0][1]),
            float(feet[1][0]), float(feet[1][1]),
            float(contacts[0]), float(contacts[1]),
            float(self.base_x), float(self.hip_z),
        ]
        self.metric_pub.publish(metrics)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
