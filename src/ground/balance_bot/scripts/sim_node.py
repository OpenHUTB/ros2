#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自适应平衡仿真节点（ROS 1）：MuJoCo 物理 + 平衡控制器。

发布：
    /imu           sensor_msgs/Imu           倾角（姿态四元数）+ 角速度 + 线加速度
    /joint_states  sensor_msgs/JointState    左右轮角度与角速度（编码器）
    /odom          nav_msgs/Odometry         由轮子推算的里程计
    /balance_debug geometry_msgs/PoseStamped 真值位姿（仅用于评价，控制回路不读）
订阅：
    /cmd_vel       geometry_msgs/Twist       linear.x = 目标速度, angular.z = 目标转向率

控制器二选一（参数 ~controller）：
    pd  —— 倾角 PD + 速度环（默认）
    nn  —— 神经网络策略（需先跑 tools/train_policy.py 生成 results/balance_policy.npz）
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

try:
    from balance_bot import config
    from balance_bot.control import BalanceController
    from balance_bot.env import BalanceBotEnv
    from balance_bot.policy_nn import PolicyMLP, StandardScaler, state_features
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from balance_bot import config
    from balance_bot.control import BalanceController
    from balance_bot.env import BalanceBotEnv
    from balance_bot.policy_nn import PolicyMLP, StandardScaler, state_features


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class SimNode(object):

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", config.CONTROL_HZ))
        self.kind = str(rospy.get_param("~controller", "pd")).lower()
        self.env = BalanceBotEnv(config.resolve_scene_path(None))
        self.pd = BalanceController()
        self.net = self.scaler = None
        if self.kind == "nn":
            path = config.DEFAULT_OUTPUT_DIR / "balance_policy.npz"
            spath = config.DEFAULT_OUTPUT_DIR / "balance_scaler.npz"
            if not path.exists():
                rospy.logwarn("找不到 %s，回退到 pd 控制器（先跑 tools/train_policy.py）" % path)
                self.kind = "pd"
            else:
                self.net = PolicyMLP.load(str(path))
                self.scaler = StandardScaler.load(str(spath))
                rospy.loginfo("已加载神经网络策略")

        self.v_target = 0.0
        self.w_target = 0.0
        self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=10)
        self.pub_js = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pub_truth = rospy.Publisher("/balance_debug", PoseStamped, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=10)
        rospy.loginfo("平衡车仿真启动（控制器=%s，%s）", self.kind,
                      os.path.basename(config.resolve_scene_path(None)))

    def on_cmd(self, msg):
        self.v_target = float(msg.linear.x)
        self.w_target = float(msg.angular.z)

    def _torque(self, st):
        if self.kind == "nn" and self.net is not None:
            x = self.scaler.transform(state_features(st, self.v_target).reshape(1, -1))
            tau = float(self.net.predict(x)[0, 0])
            tau = max(-config.TORQUE_MAX, min(config.TORQUE_MAX, tau))
            return tau, tau
        return self.pd(st, self.v_target, self.w_target)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        if not self.env.is_upright():
            self.env.reset()
        while not rospy.is_shutdown():
            st = self.env.state()
            tl, tr = self._torque(st)
            self.env.set_torque(tl, tr)
            st = self.env.step()
            stamp = rospy.Time.now()

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "base_link"
            q = yaw_to_quat(st.yaw)
            imu.orientation.z, imu.orientation.w = q[2], q[3]
            imu.angular_velocity.z = st.w
            imu.linear_acceleration.x = st.v
            imu.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
            imu.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self.pub_imu.publish(imu)

            js = JointState()
            js.header.stamp = stamp
            js.name = ["wheel_l_joint", "wheel_r_joint"]
            ang = self.env.wheel_angles()
            js.position = [ang[0], ang[1]]
            js.velocity = [st.wheel[0], st.wheel[1]]
            self.pub_js.publish(js)

            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = st.x
            od.pose.pose.position.y = st.y
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = q[2], q[3]
            od.twist.twist.linear.x = st.v
            od.twist.twist.angular.z = st.w
            self.pub_odom.publish(od)

            gt = PoseStamped()
            gt.header.stamp = stamp
            gt.header.frame_id = "world"
            gt.pose.position.x = st.x
            gt.pose.position.y = st.y
            gt.pose.position.z = st.z
            gt.pose.orientation.z, gt.pose.orientation.w = q[2], q[3]
            self.pub_truth.publish(gt)

            if not self.env.is_upright():
                rospy.logwarn("倾角超过 %.0f°，已复位（说明控制器没撑住）"
                              % math.degrees(config.FALL_ANGLE))
                self.env.reset()
            rate.sleep()


def main():
    rospy.init_node("sim_balance")
    SimNode().spin()


if __name__ == "__main__":
    main()
