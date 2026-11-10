"""MuJoCo 仿真节点：推进物理、发布机器人状态、接收关节目标。"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .sim_core import G1Sim, LEG_JOINTS, NOMINAL_LEG_POSE, UPPER_JOINTS, SimConfig
from .balance import ComBalance
from .g1_env import RESIDUAL_SCALE, TARGET_HEIGHT


def _yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SimNode(Node):
    def __init__(self):
        super().__init__("humanoid_sim_node")

        self.declare_parameter("model_path", "models/g1/scene_29dof.xml")
        self.declare_parameter("sim_rate", 50.0)
        self.declare_parameter("publish_tf", True)
        # 上半身保持增益。实测必须与训练时一致（kp=40）：调到 120 时整个系统变刚、
        # 与腿部平衡基元耦合振荡，0.8 s 就倒地。kp=40 下 1.5 rad 的抬臂目标
        # 实际能到约 1.0 rad（57°），视觉上已经很明显。
        self.declare_parameter("upper_kp", 40.0)
        self.declare_parameter("upper_kd", 1.5)

        model_path = self.get_parameter("model_path").value
        rate = float(self.get_parameter("sim_rate").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.sim = G1Sim(SimConfig(model_path=model_path))
        # 上半身保持增益；调硬一些让键盘摆臂/弯腰的动作更明显
        self.sim.upper_kp = float(self.get_parameter("upper_kp").value)
        self.sim.upper_kd = float(self.get_parameter("upper_kd").value)
        # 平衡基元：把「不摔倒」从策略里剥离出来，策略只输出残差
        self.balance = ComBalance(q_min=self.sim.leg_qmin, q_max=self.sim.leg_qmax)
        self.residual = np.zeros(len(LEG_JOINTS))
        self.upper_targets = np.zeros(len(UPPER_JOINTS))
        self.targets = NOMINAL_LEG_POSE.copy()
        self.cmd = np.zeros(2)
        self.phase = 0.0

        # 发布
        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_obs = self.create_publisher(Float64MultiArray, "humanoid/obs", 10)

        # 订阅
        self.create_subscription(
            Float64MultiArray, "humanoid/joint_targets", self._on_targets, 10
        )
        self.create_subscription(
            Float64MultiArray, "humanoid/upper_body", self._on_upper, 10
        )
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.joint_names = [
            n for n in self.sim.joint_names if n and n != "floating_base_joint"
        ]

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"G1 仿真节点已启动：模型 {model_path}，控制频率 {rate:.0f} Hz"
        )

    # ---------- 回调 ----------
    def _on_targets(self, msg: Float64MultiArray) -> None:
        """接收 12 维残差动作（-1~1），不是最终关节角。"""
        data = np.asarray(msg.data, dtype=np.float64)
        if data.size == len(LEG_JOINTS):
            self.residual = np.clip(data, -1.0, 1.0)

    def _on_upper(self, msg: Float64MultiArray) -> None:
        """接收 17 维上半身目标角（弧度），顺序同 UPPER_JOINTS。"""
        data = np.asarray(msg.data, dtype=np.float64)
        if data.size == len(UPPER_JOINTS):
            self.upper_targets = np.clip(data, self.sim.upper_qmin, self.sim.upper_qmax)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = np.array([msg.linear.x, msg.angular.z])

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        # 最终目标角 = 平衡基元输出 + 残差
        self.targets = self.balance.targets(
            NOMINAL_LEG_POSE,
            self.sim.com_error(),
            float(self.sim.com_vel()[0]),
        ) + RESIDUAL_SCALE * self.residual
        self.targets = np.clip(self.targets, self.sim.leg_qmin, self.sim.leg_qmax)

        self.sim.step(self.targets, self.upper_targets)
        self.phase = (self.phase + 0.02) % 1.0
        now = self.get_clock().now().to_msg()

        # 观测（与训练时完全一致的 40 维）
        obs = np.concatenate(
            [
                self.sim.projected_gravity(),
                self.sim.base_lin_vel_body(),
                self.sim.base_ang_vel(),
                [self.sim.base_height - TARGET_HEIGHT],
                self.sim.joint_pos() - NOMINAL_LEG_POSE,
                self.sim.joint_vel(),
                self.cmd,
                [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)],
                [self.sim.com_error(), float(self.sim.com_vel()[0])],
            ]
        )
        msg_obs = Float64MultiArray()
        msg_obs.data = obs.astype(float).tolist()
        self.pub_obs.publish(msg_obs)

        # 关节状态
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        qpos = self.sim.data.qpos
        qvel = self.sim.data.qvel
        js.position = [
            float(qpos[self.sim.model.jnt_qposadr[self.sim.model.joint(n).id]])
            for n in self.joint_names
        ]
        js.velocity = [
            float(qvel[self.sim.model.jnt_dofadr[self.sim.model.joint(n).id]])
            for n in self.joint_names
        ]
        self.pub_joint.publish(js)

        # 里程计
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(qpos[0])
        odom.pose.pose.position.y = float(qpos[1])
        odom.pose.pose.position.z = float(qpos[2])
        odom.pose.pose.orientation.w = float(qpos[3])
        odom.pose.pose.orientation.x = float(qpos[4])
        odom.pose.pose.orientation.y = float(qpos[5])
        odom.pose.pose.orientation.z = float(qpos[6])
        # odom.twist 应表达在 child_frame（base_link）下
        v_body = self.sim.base_lin_vel_body()
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.angular.z = float(self.sim.base_ang_vel()[2])
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = float(qpos[0])
            tf.transform.translation.y = float(qpos[1])
            tf.transform.translation.z = float(qpos[2])
            tf.transform.rotation.w = float(qpos[3])
            tf.transform.rotation.x = float(qpos[4])
            tf.transform.rotation.y = float(qpos[5])
            tf.transform.rotation.z = float(qpos[6])
            self.tf_broadcaster.sendTransform(tf)


def main(argv=None):
    rclpy.init(args=argv)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
