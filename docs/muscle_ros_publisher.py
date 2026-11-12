#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""muscle_ros_publisher.py

把 MuJoCo 肌肉骨骼二连杆臂（muscle_arm.xml，4 条肌腱驱动肌肉）的仿真状态
实时发布到 ROS2 话题：

  /joint_states    sensor_msgs/JointState      关节角度、角速度（标准话题）
  /muscle_state    std_msgs/Float64MultiArray  4 条肌肉的激活度
  /muscle_force    std_msgs/Float64MultiArray  4 条肌肉的执行器力

运行方式（不需 colcon 构建，全部使用标准消息类型）：

    python3 muscle_ros_publisher.py

依赖：mujoco 3.2.3、rclpy（ROS2 Humble）、numpy
"""

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

XML_PATH = "muscle_arm.xml"
DT = 0.002  # 仿真步长 2 ms（名义 500 Hz）


class MuscleArmBridge(Node):
    def __init__(self):
        super().__init__("muscle_arm_bridge")
        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.data = mujoco.MjData(self.model)
        self.n_joint = self.model.njnt

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ms_pub = self.create_publisher(Float64MultiArray, "/muscle_state", 10)
        self.mf_pub = self.create_publisher(Float64MultiArray, "/muscle_force", 10)

        self.t = 0.0
        self.create_timer(DT, self.step)
        self.get_logger().info(
            "muscle_arm.xml 已加载：%d 个关节、%d 个肌肉执行器"
            % (self.model.njnt, self.model.nu))

    def ctrl(self, t):
        """前 1.5 s 激励伸肌使手臂伸展，之后切到屈肌使手臂屈曲"""
        c = np.zeros(self.model.nu)
        if t < 1.5:
            c[1::2] = 0.8  # 伸肌
        else:
            c[0::2] = 0.8  # 屈肌
        return c

    def step(self):
        self.data.ctrl[:] = self.ctrl(self.t)
        mujoco.mj_step(self.model, self.data)
        self.t += DT

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.model.joint(i).name for i in range(self.n_joint)]
        js.position = [
            self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.n_joint)]
        js.velocity = [
            self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.n_joint)]
        self.js_pub.publish(js)

        act = Float64MultiArray()
        act.data = [float(self.data.act[i]) for i in range(self.model.na)]
        self.ms_pub.publish(act)

        force = Float64MultiArray()
        force.data = [float(v) for v in self.data.actuator_force]
        self.mf_pub.publish(force)


def main():
    rclpy.init()
    node = MuscleArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
