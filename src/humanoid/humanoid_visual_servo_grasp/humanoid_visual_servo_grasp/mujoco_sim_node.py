"""
humanoid_visual_servo_grasp ROS2 仿真节点
=========================================
在 MuJoCo 中实时仿真「人形单臂视觉伺服抓取」，周期发布抓取状态话题：
    /humanoid/grasp/status     末端误差、是否到达、是否抓取完成（Float64MultiArray）
    /humanoid/grasp/target     目标物位置（PoseStamped）
    /humanoid/grasp/tip        末端位置（PoseStamped）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from humanoid_visual_servo_grasp.sim_core import (
    ServoGraspController, CONTROL_HZ, REACH_TOL)

import mujoco


class VisualServoGraspNode(Node):
    def __init__(self):
        super().__init__('visual_servo_grasp_node')
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('visualize', False)

        self.ctrl = ServoGraspController()
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.dt = 1.0 / self.control_hz
        self.n_sub = max(1, int(round(self.dt / self.ctrl.model.opt.timestep)))

        self.status_pub = self.create_publisher(Float64MultiArray, '/humanoid/grasp/status', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/humanoid/grasp/target', 10)
        self.tip_pub = self.create_publisher(PoseStamped, '/humanoid/grasp/tip', 10)

        self.viewer = None
        if self.get_parameter('visualize').value:
            self.viewer = mujoco.viewer.launch_passive(self.ctrl.model, self.ctrl.data)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            'visual_servo_grasp_node 已启动，控制频率 %.1f Hz' % self.control_hz)

    def _pose(self, xyz):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(xyz[0])
        p.pose.position.y = float(xyz[1])
        p.pose.position.z = float(xyz[2])
        p.pose.orientation.w = 1.0
        return p

    def timer_callback(self):
        err, reached, grasped = self.ctrl.servo_step(self.dt)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.ctrl.model, self.ctrl.data)
        if self.viewer is not None:
            self.viewer.sync()

        # 抓取状态（误差/到达/抓取完成）
        status = Float64MultiArray()
        status.layout.dim = [MultiArrayDimension(label='grasp', size=3, stride=3)]
        status.data = [float(err), float(1.0 if reached else 0.0),
                       float(1.0 if grasped else 0.0)]
        self.status_pub.publish(status)

        # 目标物与末端位置
        self.target_pub.publish(self._pose(self.ctrl.object_pos()))
        self.tip_pub.publish(self._pose(self.ctrl.tip_pos()))

        if grasped:
            self.get_logger().info('视觉伺服抓取完成（误差 %.3f m）' % err)

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
