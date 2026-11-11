"""
humanoid_human_following ROS2 仿真节点
======================================
在 MuJoCo 中实时仿真「人形跟随移动目标」，周期发布跟随状态话题：
    /humanoid/follow/target    目标位置（PoseStamped）
    /humanoid/follow/status    距离、速度、转向、是否停稳（自定义 Float64MultiArray）
    /humanoid/odom             机器人里程计（Odometry）
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from humanoid_human_following.sim_core import FollowController, CONTROL_HZ, D_KEEP

import mujoco
import numpy as np


class HumanoidFollowNode(Node):
    def __init__(self):
        super().__init__('humanoid_follow_node')
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('d_keep', D_KEEP)
        self.declare_parameter('visualize', False)

        self.ctrl = FollowController()
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.dt = 1.0 / self.control_hz
        self.n_sub = max(1, int(round(self.dt / self.ctrl.model.opt.timestep)))

        # 话题发布器
        self.target_pub = self.create_publisher(PoseStamped, '/humanoid/follow/target', 10)
        self.status_pub = self.create_publisher(Float64MultiArray, '/humanoid/follow/status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/humanoid/odom', 10)

        self.viewer = None
        if self.get_parameter('visualize').value:
            self.viewer = mujoco.viewer.launch_passive(self.ctrl.model, self.ctrl.data)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            'humanoid_follow_node 已启动，跟随距离 %.2f m，控制频率 %.1f Hz' % (D_KEEP, self.control_hz))

    def timer_callback(self):
        # 推进一个控制周期（含多个物理子步）
        d, v, w, e_yaw = self.ctrl.follow_step(self.dt)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.ctrl.model, self.ctrl.data)
        if self.viewer is not None:
            self.viewer.sync()

        # 发布目标位置
        tgt = self.ctrl.target_pos()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(tgt[0])
        pose.pose.position.y = float(tgt[1])
        pose.pose.position.z = float(tgt[2])
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)

        # 发布跟随状态（距离/速度/转向/是否停稳）
        status = Float64MultiArray()
        status.layout.dim = [MultiArrayDimension(label='follow', size=4, stride=4)]
        status.data = [float(d), float(v), float(w), float(1.0 if abs(d - D_KEEP) < 0.25 else 0.0)]
        self.status_pub.publish(status)

        # 发布里程计
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(self.ctrl.data.qpos[self.ctrl.root_qadr])
        odom.pose.pose.position.y = float(self.ctrl.data.qpos[self.ctrl.root_qadr + 1])
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
