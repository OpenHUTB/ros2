#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 演示用发布器：同时发布"真值轨迹"与"带误差的里程计轨迹"。

这样不装任何仿真器（Gazebo / CARLA）也能把 ``traj_eval_node`` 跑起来：
一条 ``roslaunch trajectory_accuracy_eval main.launch`` 就能看到
ATE / RPE 随行驶距离增长的过程。

参数（见 ``config/params.yaml``）：``kind`` / ``speed`` / ``duration`` /
``wheel_scale`` / ``yaw_bias_deg_per_m`` / ``rate_hz`` / ``loop``。
"""

from __future__ import annotations

import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from traj_eval import OdomErrorModel, make_ground_truth, simulate_odometry


def _quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _to_msg(t: float, x: float, y: float, yaw: float) -> Odometry:
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    qx, qy, qz, qw = _quat_from_yaw(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.angular.z = 0.0
    _ = t
    return msg


class SimOdomPublisher:
    """把预先生成的两条轨迹按固定频率回放。"""

    def __init__(self) -> None:
        rospy.init_node("sim_odom_publisher")

        kind = str(rospy.get_param("~kind", "figure8"))
        speed = float(rospy.get_param("~speed", 1.5))
        duration = float(rospy.get_param("~duration", 40.0))
        wheel_scale = float(rospy.get_param("~wheel_scale", 1.02))
        yaw_bias = float(rospy.get_param("~yaw_bias_deg_per_m", 0.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.loop = bool(rospy.get_param("~loop", True))

        self.gt = make_ground_truth(kind, speed=speed, duration=duration, dt=1.0 / self.rate_hz)
        self.est = simulate_odometry(
            self.gt, OdomErrorModel(wheel_scale=wheel_scale, yaw_bias_deg_per_m=yaw_bias))

        self.pub_gt = rospy.Publisher("/ground_truth", Odometry, queue_size=10)
        self.pub_est = rospy.Publisher("/odom", Odometry, queue_size=10)
        # 循环回放时通知评估节点清空缓冲：否则两圈轨迹会攒在一起，
        # 循环处的"瞬间跳回起点"会被当成真实运动，把 RPE 抬高好几倍
        self.pub_reset = rospy.Publisher("/traj_eval/reset", Empty, queue_size=1)
        self.index = 0

        rospy.loginfo("sim_odom_publisher: kind=%s speed=%.2f m/s wheel_scale=%.3f，"
                      "共 %d 个采样点", kind, speed, wheel_scale, len(self.gt))
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

    def _tick(self, _event) -> None:
        if self.index >= len(self.gt):
            if not self.loop:
                rospy.loginfo("轨迹回放结束（loop=false），可 Ctrl+C 退出")
                return
            self.index = 0
            self.pub_reset.publish(Empty())
            rospy.loginfo("轨迹回放循环到起点，已通知评估节点重置")

        g = self.gt[self.index]
        e = self.est[self.index]
        self.pub_gt.publish(_to_msg(0.0, g.x, g.y, g.yaw))
        self.pub_est.publish(_to_msg(0.0, e.x, e.y, e.yaw))
        self.index += 1


def main() -> None:
    SimOdomPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
