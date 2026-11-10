"""MuJoCo 仿真节点：推进物理、发布机器人状态、接收策略动作。

节点只做「物理 + 观测」，不含任何神经网络推理，因此可以单独替换策略
（见 :mod:`humanoid_h1_walk.policy_node`）。话题接口：

============================  ======================================
``/joint_states``             腿部关节角与角速度（sensor_msgs/JointState）
``/odom``                     骨盆位姿与机体系速度（nav_msgs/Odometry）
``/humanoid/obs``             策略输入向量（std_msgs/Float64MultiArray）
``/cmd_vel``                  速度指令 [vx, vy, wz]（geometry_msgs/Twist）
``/humanoid/joint_targets``   策略输出的动作（std_msgs/Float64MultiArray）
============================  ======================================
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .walker import Walker, load_specs, routine_command


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("humanoid_h1_walk_sim")

        self.declare_parameter("robot", "h1")
        self.declare_parameter("config", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("routine", False)
        self.declare_parameter("policy_in_sim", False)

        robot = str(self.get_parameter("robot").value)
        config = str(self.get_parameter("config").value) or None
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.routine = bool(self.get_parameter("routine").value)
        # policy_in_sim=true 时在本节点内直接推理，省掉一次话题往返（等价于上游
        # deploy_mujoco.py 的单进程行为）；默认 false，走 /humanoid/obs → 策略节点
        self.policy_in_sim = bool(self.get_parameter("policy_in_sim").value)

        specs = load_specs(config)
        self.walker = Walker(robot, specs, load_policy=self.policy_in_sim)
        spec = self.walker.spec
        self.cmd = np.zeros(3, dtype=np.float32)
        self.rate = 1.0 / spec.control_dt

        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_obs = self.create_publisher(Float64MultiArray, "humanoid/obs", 10)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_subscription(
            Float64MultiArray, "humanoid/joint_targets", self._on_action, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.joint_names = self.walker.joint_names
        self._label = ""

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"{robot} 仿真节点已启动：{spec.description}，{spec.num_actions} 个关节，"
            f"控制频率 {self.rate:.0f} Hz"
            + ("（固定动作序列）" if self.routine else "")
        )

    # ---------- 回调 ----------
    def _on_cmd(self, msg: Twist) -> None:
        if self.routine:
            return
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)

    def _on_action(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size == self.walker.spec.num_actions:
            self.walker.apply_action(data)

    # ---------- 主循环 ----------
    def _tick(self) -> None:
        if self.routine:
            self.cmd, label = routine_command(self.walker.data.time)
            if label != self._label:
                self._label = label
                self.get_logger().info(f"阶段：{label}")

        # 1) 用上一帧的目标角推进物理
        self.walker.advance()

        # 2) 组观测：策略节点据此算动作，或在本节点内直接推理
        obs = self.walker.observe(self.cmd)
        if self.policy_in_sim:
            self.walker.apply_action(self.walker.act(obs))

        msg_obs = Float64MultiArray()
        msg_obs.data = obs.astype(float).tolist()
        self.pub_obs.publish(msg_obs)

        # 3) 发布状态
        now = self.get_clock().now().to_msg()
        self._publish_joint_state(now)
        self._publish_odom(now)

    def _publish_joint_state(self, stamp) -> None:
        w = self.walker
        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        model, data = w.model, w.data
        js.position = [
            float(data.qpos[model.jnt_qposadr[model.joint(n).id]]) for n in self.joint_names
        ]
        js.velocity = [
            float(data.qvel[model.jnt_dofadr[model.joint(n).id]]) for n in self.joint_names
        ]
        self.pub_joint.publish(js)

    def _publish_odom(self, stamp) -> None:
        w = self.walker
        qpos = w.data.qpos
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(qpos[0])
        odom.pose.pose.position.y = float(qpos[1])
        odom.pose.pose.position.z = float(qpos[2])
        odom.pose.pose.orientation.w = float(qpos[3])
        odom.pose.pose.orientation.x = float(qpos[4])
        odom.pose.pose.orientation.y = float(qpos[5])
        odom.pose.pose.orientation.z = float(qpos[6])
        v_body = w.base_lin_vel_body()
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        odom.twist.twist.angular.z = float(w.data.qvel[5])
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is None:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
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


def main(argv=None) -> int:
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
