# -*- coding: utf-8 -*-
"""ROS 2 节点：按参数跑一遍爬台阶，并把关节角与基座位姿发出去。

发布：
    /stair_climb/joint_states   sensor_msgs/JointState   六个关节角
    /stair_climb/odom           nav_msgs/Odometry        基座位姿
    /stair_climb/status         std_msgs/String          进度与最终指标
    /tf                         tf2_msgs/TFMessage       odom -> base_link

指标口径与 `main.py --headless` 一致：到达平台后按 hold 秒继续演示，
停下来的那一帧作为最终指标，所以「用时/步数/终点 x」两处对得上。
"""

import math
import os

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros

import humanoid_stair_climb.config as config
from humanoid_stair_climb.gait import StairGait
from humanoid_stair_climb.metrics import RunRecorder, format_summary
from humanoid_stair_climb.sim_core import StairSim

JOINTS = ("hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r")


class StairClimbNode(Node):
    def __init__(self):
        super().__init__("stair_climb_node")
        self.declare_parameter("control_hz", config.CONTROL_HZ)
        self.declare_parameter("time_limit", config.TIME_LIMIT)
        self.declare_parameter("hold", config.HOLD_TIME)
        self.declare_parameter("cycle_time", config.CYCLE_TIME)
        self.declare_parameter("swing_ratio", config.SWING_RATIO)
        self.declare_parameter("lift_height", config.LIFT_HEIGHT)
        self.declare_parameter("scene", "")

        self.hz = float(self.get_parameter("control_hz").value)
        self.time_limit = float(self.get_parameter("time_limit").value)
        self.hold = float(self.get_parameter("hold").value)
        scene = self.get_parameter("scene").value or None

        self.gait = StairGait(
            cycle_time=float(self.get_parameter("cycle_time").value),
            swing_ratio=float(self.get_parameter("swing_ratio").value),
            lift_height=float(self.get_parameter("lift_height").value),
        )
        self.sim = StairSim(scene)
        self.rec = RunRecorder()

        self.pub_js = self.create_publisher(JointState, "stair_climb/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "stair_climb/odom", 10)
        self.pub_status = self.create_publisher(String, "stair_climb/status", 10)
        # 与 /stair_climb/odom 对应，播 odom -> base_link，供 RViz 等下游使用
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.t = 0.0
        self.base_z = config.STAND_Z
        self.dt = 1.0 / self.hz
        self.done = False
        self.reached_at = None
        self.last = None          # (base_x, base_z, joints)，done 之后按这一帧继续播
        self.create_timer(self.dt, self.step)
        self.get_logger().info("stair_climb_node 启动：%.1f Hz，场景 %s" % (self.hz, self.sim.model))

    def step(self):
        if self.done:
            # 已经停在平台上：继续按固定位姿发话题，避免下游查出超时
            if self.last is not None:
                self._publish(*self.last)
            return

        self.t += self.dt
        base_x = self.gait.base_x(self.t)
        target_z = config.STAND_Z + self.gait.support_height(self.t)
        self.base_z += (target_z - self.base_z) * min(1.0, self.dt / config.BASE_Z_TAU)
        joints = self.gait.joint_targets(self.t, base_x, self.base_z)
        self.sim.set_state(base_x, self.base_z, joints)

        # 与 main.py 一致：先记落脚点再记采样，否则步数恒为 0
        states = {side: self.gait.foot_state(self.t, side) for side in ("l", "r")}
        self.rec.detect_landings(self.t, states, self.sim)
        self.rec.add(self.t, self.sim.base_pos(), joints,
                     self.sim.clearance(), self.gait.support_height(self.t))

        self.last = (base_x, self.base_z, joints)
        self._publish(base_x, self.base_z, joints)

        if self.reached_at is None and self.sim.reached():
            self.reached_at = self.t
        finished = ((self.reached_at is not None and self.t - self.reached_at >= self.hold)
                    or self.t >= self.time_limit)
        if finished:
            self.done = True
            summary = self.rec.summary()
            self.pub_status.publish(String(data=format_summary(summary)))
            self.get_logger().info(format_summary(summary))

    def _publish(self, base_x, base_z, joints):
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(JOINTS)
        js.position = list(joints)
        self.pub_js.publish(js)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = base_x
        odom.pose.pose.position.z = base_z
        odom.pose.pose.orientation = Quaternion(w=1.0)
        self.pub_odom.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = base_x
        tf_msg.transform.translation.z = base_z
        tf_msg.transform.rotation = Quaternion(w=1.0)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StairClimbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
