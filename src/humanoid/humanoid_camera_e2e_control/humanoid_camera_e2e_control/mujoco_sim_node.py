# -*- coding: utf-8 -*-
"""
mujoco_sim_node.py —— 人形机器人 MuJoCo 仿真 ROS2 节点。

加载避障场景人形模型，按 /cmd_vel 速度指令推进物理仿真，发布：
    /humanoid/odom             nav_msgs/Odometry
    /joint_states              sensor_msgs/JointState
    /tf                        world -> humanoid/base_link
    /humanoid/camera/image_raw sensor_msgs/Image（跟随视角 RGB8，端到端策略输入）

控制策略见 sim_core.py：站姿 PD + 速度级前进 + 姿态级转向。
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image
from tf2_ros import TransformBroadcaster

try:
    from humanoid_camera_e2e_control.sim_core import HumanoidSim, open_viewer, IMG_H, IMG_W
except ImportError:
    from sim_core import HumanoidSim, open_viewer, IMG_H, IMG_W


class MujocoSimNode(Node):
    """人形机器人 MuJoCo 仿真节点。"""

    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 500.0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('image_rate', 10.0)
        self.declare_parameter('kp', 120.0)
        self.declare_parameter('kv', 6.0)
        self.declare_parameter('kp_roll', 400.0)
        self.declare_parameter('v_max', 0.30)
        self.declare_parameter('w_max', 0.60)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('visualize', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        image_rate = self.get_parameter('image_rate').get_parameter_value().double_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

        self.sim = HumanoidSim(model_path=model_path or None)
        self.sim.KP = self.get_parameter('kp').get_parameter_value().double_value
        self.sim.KV = self.get_parameter('kv').get_parameter_value().double_value
        self.sim.KP_ROLL = self.get_parameter('kp_roll').get_parameter_value().double_value
        self.sim.V_MAX = self.get_parameter('v_max').get_parameter_value().double_value
        self.sim.W_MAX = self.get_parameter('w_max').get_parameter_value().double_value

        self.viewer = open_viewer(self.sim, self.get_logger()) if self.visualize else None
        self.viewer_timer = None
        if self.viewer is not None:
            self.viewer_timer = self.create_timer(1.0 / 60.0, self._viewer_callback)

        self.cmd_vel = Twist()
        self.last_cmd_time = None

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/humanoid/odom', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_image = self.create_publisher(Image, '/humanoid/camera/image_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        sim_period = 1.0 / sim_rate
        self.steps_per_publish = max(1, int(sim_rate / publish_rate))
        self.steps_per_image = max(1, int(sim_rate / image_rate))
        self.step_count = 0

        self.sim_timer = self.create_timer(sim_period, self._sim_step)
        self.pub_timer = self.create_timer(1.0 / publish_rate, self._publish_state)
        self.image_timer = self.create_timer(1.0 / image_rate, self._publish_image)

        self.get_logger().info(
            f'人形仿真节点已启动 | 物理 {sim_rate:.0f}Hz | 状态 {publish_rate:.0f}Hz | '
            f'图像 {image_rate:.0f}Hz | 模型 {model_path or "(包内默认)"}')

    def _cmd_callback(self, msg: Twist):
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def _sim_step(self):
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.cmd_vel = Twist()
        self.sim.step(v_lin=self.cmd_vel.linear.x, w_yaw=self.cmd_vel.angular.z)
        self.step_count += 1

    def _publish_state(self):
        now = self.get_clock().now().to_msg()
        od = self.sim.read_odom()
        quat = self.sim.data.qpos[self.sim.root_qadr + 3: self.sim.root_qadr + 7]

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'humanoid/base_link'
        odom.pose.pose.position.x = od['x']
        odom.pose.pose.position.y = od['y']
        odom.pose.pose.position.z = od['z']
        odom.pose.pose.orientation.w = float(quat[0])
        odom.pose.pose.orientation.x = float(quat[1])
        odom.pose.pose.orientation.y = float(quat[2])
        odom.pose.pose.orientation.z = float(quat[3])
        odom.twist.twist.linear.x = od['vx']
        odom.twist.twist.linear.y = od['vy']
        odom.twist.twist.angular.z = od['wz']
        self.pub_odom.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'humanoid/base_link'
        tf_msg.transform.translation.x = od['x']
        tf_msg.transform.translation.y = od['y']
        tf_msg.transform.translation.z = od['z']
        tf_msg.transform.rotation.w = float(quat[0])
        tf_msg.transform.rotation.x = float(quat[1])
        tf_msg.transform.rotation.y = float(quat[2])
        tf_msg.transform.rotation.z = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)

        jp = self.sim.read_joint_positions()
        jstate = JointState()
        jstate.header.stamp = now
        jstate.name = list(jp.keys())
        jstate.position = list(jp.values())
        self.pub_joint.publish(jstate)

        if self.step_count % (self.steps_per_publish * 30) == 0:
            self.get_logger().info(
                f'pos=({od["x"]:.2f},{od["y"]:.2f},{od["z"]:.2f}) '
                f'yaw={np.degrees(od["yaw"]):6.1f}° 站立={self.sim.standing()}')

    def _publish_image(self):
        now = self.get_clock().now().to_msg()
        img = self.sim.render_front()
        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = 'humanoid/follow_camera'
        msg.height, msg.width = IMG_H, IMG_W
        msg.encoding = 'rgb8'
        msg.step = IMG_W * 3
        msg.data = img.tobytes()
        self.pub_image.publish(msg)

    def _viewer_callback(self):
        if self.viewer is None:
            return
        try:
            self.viewer.sync()
            if not self.viewer.is_running():
                self.get_logger().info('可视化窗口已关闭，仿真后台继续运行。')
                self.close_viewer()
        except Exception as exc:                                      # noqa: BLE001
            self.get_logger().warning(f'可视化窗口异常，停止刷新：{exc}')
            self.close_viewer()

    def close_viewer(self):
        if self.viewer_timer is not None:
            try:
                self.viewer_timer.cancel()
            except Exception:                                         # noqa: BLE001
                pass
            self.viewer_timer = None
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:                                         # noqa: BLE001
                pass
            self.viewer = None


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_viewer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
