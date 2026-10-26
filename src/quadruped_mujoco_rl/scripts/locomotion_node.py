#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四足机器人运动控制 ROS 2 节点（rclpy）。

原模块只有 ROS 1（Noetic）入口，在 ROS 2 Humble 上无法运行；本节点把
``locomotion_core`` 的站立/步态控制器接进 ROS 2：

    发布  /joint_states   sensor_msgs/JointState（12 个关节角与角速度）
    订阅  /cmd_vel        geometry_msgs/Twist（linear.x = 期望前进速度，m/s）
    发布  /locomotion_status  std_msgs/String（"stand" / "trot"，便于观察）

说明：订阅到 ``|linear.x| > 0.02`` 时切换到对角步态，并用速度指令线性缩放
步态幅度（步态频率保持参数值）；否则回到站立保持。仿真在节点定时器里
按 MuJoCo 时间步推进，节点不依赖 Gazebo。

运行（无需 colcon 构建，直接以脚本方式跑）：::

    source /opt/ros/humble/setup.bash
    python3 scripts/locomotion_node.py                       # 默认步态
    python3 scripts/locomotion_node.py --ros-args -p mode:=stand
    python3 scripts/locomotion_node.py --ros-args -p control_hz:=50.0

另开终端观察::

    ros2 topic hz /joint_states
    ros2 topic echo /locomotion_status
    ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.6}}"
"""

from __future__ import annotations

import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# 与 locomotion_core 同目录，直接以脚本方式运行时可以 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from locomotion_core import (  # noqa: E402
    JOINT_PARTS,
    LEG_ORDER,
    QuadrupedModel,
    StandController,
    TrotGait,
    base_rpy_deg,
    base_height,
    default_model_path,
)

HINGE_JOINTS = tuple(
    "%s_%s_joint" % (leg, part) for leg in LEG_ORDER for part in JOINT_PARTS
)

REF_SPEED = 0.8             # 标称幅度（thigh 0.15 / calf 0.29）下的实测速度 (m/s)
SPEED_TO_AMPLITUDE = 1.0 / REF_SPEED   # m/s → 步态幅度缩放（1.0 对应默认幅度）
MAX_SPEED = 1.2             # 速度指令上限 (m/s)


class LocomotionNode(Node):
    def __init__(self):
        super().__init__("quadruped_locomotion")

        self.declare_parameter("model_path", default_model_path())
        self.declare_parameter("mode", "trot")          # trot / stand
        self.declare_parameter("freq", 3.97)
        self.declare_parameter("thigh_amp", 0.15)
        self.declare_parameter("calf_amp", 0.29)
        self.declare_parameter("phase_offset", math.pi / 2)
        self.declare_parameter("kp_roll", 0.01)
        self.declare_parameter("kp_pitch", 0.0)
        self.declare_parameter("kp_height", 0.0)
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("publish_joint_state", True)

        model_path = self.get_parameter("model_path").value
        self.robot = QuadrupedModel(model_path)
        self.get_logger().info(
            "模型: %s（nq=%d, nu=%d, 控制方式: 位置伺服目标关节角）"
            % (model_path, self.robot.model.nq, self.robot.model.nu))

        self._mode = str(self.get_parameter("mode").value)
        self._speed_cmd = 0.0
        self._t = 0.0        # 仿真总时间（日志用）
        self._gait_t = 0.0   # 步态相位计时，每次切到 trot 时归零，保证可复现
        self._steps_per_tick = max(
            1, int(round(1.0 / float(self.get_parameter("control_hz").value)
                         / self.robot.timestep)))
        self._controller = None
        self._activity = None

        self._joint_names = []
        self._qpos_adr = []
        self._dof_adr = []
        for name in HINGE_JOINTS:
            jid = next(
                (i for i in range(self.robot.model.njnt)
                 if self._joint_name(i) == name), None)
            if jid is None:
                self.get_logger().warn("未找到关节 %s，跳过发布" % name)
                continue
            self._joint_names.append(name)
            self._qpos_adr.append(int(self.robot.model.jnt_qposadr[jid]))
            self._dof_adr.append(int(self.robot.model.jnt_dofadr[jid]))

        self._pub_js = None
        if bool(self.get_parameter("publish_joint_state").value):
            self._pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_status = self.create_publisher(String, "/locomotion_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_timer(1.0 / float(self.get_parameter("control_hz").value),
                          self._tick)

        self.get_logger().info(
            "已启动：发布 /joint_states 与 /locomotion_status，订阅 /cmd_vel"
            "（linear.x = 期望速度，|v|>0.02 时走动，否则站立）")

    def _joint_name(self, jid):
        import mujoco
        return mujoco.mj_id2name(self.robot.model, mujoco.mjtObj.mjOBJ_JOINT, jid)

    def _build_controller(self, mode, amplitude_scale):
        if mode == "stand":
            return StandController(self.robot)
        return TrotGait(
            self.robot,
            freq=float(self.get_parameter("freq").value),
            a_thigh=float(self.get_parameter("thigh_amp").value) * amplitude_scale,
            a_calf=float(self.get_parameter("calf_amp").value) * amplitude_scale,
            phase_offset=float(self.get_parameter("phase_offset").value),
            kp_roll=float(self.get_parameter("kp_roll").value),
            kp_pitch=float(self.get_parameter("kp_pitch").value),
            kp_height=float(self.get_parameter("kp_height").value),
        )

    def _cmd_vel_cb(self, msg):
        self._speed_cmd = max(-MAX_SPEED, min(MAX_SPEED, float(msg.linear.x)))

    def _amplitude_scale(self):
        """速度指令 → 步态幅度缩放（0.2 ~ 2.0 倍）。"""
        return min(2.0, max(0.2, abs(self._speed_cmd) * SPEED_TO_AMPLITUDE))

    def _tick(self):
        import mujoco

        # 决定本周期用哪种控制：stand 模式恒定站立，trot 模式随速度指令切换
        if self._mode == "stand":
            desired = "stand"
        else:
            desired = "trot" if abs(self._speed_cmd) > 0.02 else "stand"
        if desired != self._activity:
            self._activity = desired
            if desired == "stand":
                self._controller = StandController(self.robot)
            else:
                self._gait_t = 0.0   # 相位归零：保证每次起步的步态可复现
                self._controller = self._build_controller(
                    "trot", self._amplitude_scale())
                p = self._controller.params()
                self.get_logger().info(
                    "步态参数: freq=%.2f Hz, thigh=%.3f rad, calf=%.3f rad, "
                    "phase_offset=%.2f, kp_roll=%s"
                    % (p["freq"], p["a_thigh"], p["a_calf"],
                       p["phase_offset"], p["kp_roll"]))
            self.get_logger().info("控制模式切换为: %s" % desired)

        # 推进仿真：一个定时器周期内跑 steps_per_tick 个 MuJoCo 步
        for _ in range(self._steps_per_tick):
            self._controller.update(self._gait_t)
            mujoco.mj_step(self.robot.model, self.robot.data)
            self._t += self.robot.timestep
            self._gait_t += self.robot.timestep

        if self._pub_js is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self._joint_names)
            msg.position = [float(self.robot.data.qpos[a]) for a in self._qpos_adr]
            msg.velocity = [float(self.robot.data.qvel[a]) for a in self._dof_adr]
            self._pub_js.publish(msg)

        roll, pitch = base_rpy_deg(self.robot.model, self.robot.data)
        status = String()
        status.data = "%s |cmd_v|=%.2f m/s | x=%.2f y=%.2f z=%.3f | tilt=%.1f°" % (
            self._activity, self._speed_cmd,
            float(self.robot.data.qpos[0]), float(self.robot.data.qpos[1]),
            base_height(self.robot.data), math.hypot(roll, pitch))
        self._pub_status.publish(status)
        self.get_logger().info(status.data, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
