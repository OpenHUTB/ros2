#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mujoco_musculoskeletal_knee_tracking 主节点 (ROS 2 Humble, ament_python)

功能:
  1. 加载单关节膝关节肌肉骨骼模型 (MuJoCo MJCF)
  2. 订阅 /knee/reference (std_msgs/Float64, 单位: 度) 接收目标角度;
     若无外部消息则按参数内置正弦/阶跃轨迹
  3. PD 控制器将跟踪误差映射为拮抗肌肉激活 (flexor/extensor ∈ [0,1])
  4. 发布关节角 / 激活 / 实际广义力矩

运行:
  colcon build 后:  ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py
"""
import math
import os
import threading

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, Float64MultiArray

from mujoco_musculoskeletal_knee_tracking.controller import pd_to_activation

_MODEL_REL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model",
                          "muscle_knee.xml")
try:
    import ament_index_python.packages as aip
    if "mujoco_musculoskeletal_knee_tracking" in aip.get_packages_with_prefixes():
        _MODEL_REL = os.path.join(
            aip.get_package_share_directory("mujoco_musculoskeletal_knee_tracking"),
            "model", "muscle_knee.xml")
except Exception:
    pass
MODEL_PATH = os.environ.get("KNEE_MODEL_PATH", _MODEL_REL)


class KneeTrackingNode(Node):
    def __init__(self):
        super().__init__("mujoco_musculoskeletal_knee_tracking_node")

        # ---- 参数 ----
        self.declare_parameter("model_path", MODEL_PATH)
        self.declare_parameter("sim.dt", 0.005)
        self.declare_parameter("sim.duration", 30.0)
        self.declare_parameter("trajectory.type", "sine")          # sine | step
        self.declare_parameter("trajectory.amplitude_deg", 40.0)
        self.declare_parameter("trajectory.frequency_hz", 0.4)
        self.declare_parameter("trajectory.bias_deg", 30.0)
        self.declare_parameter("ctrl.kp", 30.0)
        self.declare_parameter("ctrl.kd", 3.0)
        self.declare_parameter("ctrl.max_torque", 25.0)            # 肌肉等效最大力矩 N·m
        self.declare_parameter("ctrl.limit_deg", 90.0)             # 模型关节限位上限

        model_path = self.get_parameter("model_path").value
        self.dt = self.get_parameter("sim.dt").value
        self.duration = self.get_parameter("sim.duration").value
        self.traj_type = self.get_parameter("trajectory.type").value
        self.amp = self.get_parameter("trajectory.amplitude_deg").value
        self.freq = self.get_parameter("trajectory.frequency_hz").value
        self.bias = self.get_parameter("trajectory.bias_deg").value
        self.kp = self.get_parameter("ctrl.kp").value
        self.kd = self.get_parameter("ctrl.kd").value
        self.max_torque = self.get_parameter("ctrl.max_torque").value
        self.limit_deg = self.get_parameter("ctrl.limit_deg").value

        if self.traj_type not in ("sine", "step"):
            self.get_logger().warn(f"未知轨迹类型 {self.traj_type}, 回退为 sine")
            self.traj_type = "sine"

        # ---- 加载 MuJoCo 模型 ----
        with open(model_path, "r", encoding="utf-8") as f:
            xml = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.i_flex = self.model.actuator("flexor").id
        self.i_ext = self.model.actuator("extensor").id

        self.sim_time = 0.0
        self._ref_deg = self.bias
        self._use_external_ref = False
        self._ref_lock = threading.Lock()
        self._lock = threading.Lock()

        # ---- 话题 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Float64, "knee/reference", self._on_reference, qos)
        self._angle_pub = self.create_publisher(Float64, "knee/angle", qos)
        self._act_pub = self.create_publisher(Float64MultiArray, "knee/activation", qos)
        self._torque_pub = self.create_publisher(Float64, "knee/torque", qos)

        # 定时器: 每 0.02s 步进 4 步(50Hz 发布)
        self._steps = 4
        self.create_timer(self.dt * self._steps, self._on_tick)

        self.get_logger().info(
            f"膝关节肌肉骨骼轨迹跟踪已启动 | 模型: {model_path} | "
            f"轨迹: {self.traj_type}(amp={self.amp}deg f={self.freq}Hz "
            f"bias={self.bias}deg) | PD kp={self.kp} kd={self.kd}")

    def _on_reference(self, msg):
        """外部目标角度(deg)订阅回调, 覆盖内置轨迹"""
        with self._ref_lock:
            self._use_external_ref = True
            self._ref_deg = float(msg.data)

    def _reference_deg(self, t):
        """内置轨迹: sine 或 step"""
        if self.traj_type == "step":
            return self.bias if t < self.duration * 0.4 else self.bias + self.amp
        return self.bias + self.amp * 0.5 * (1 - math.cos(2 * math.pi * self.freq * t))

    def _on_tick(self):
        with self._lock:
            for _ in range(self._steps):
                if self.sim_time >= self.duration:
                    self._safe_stop()
                    mujoco.mj_step(self.model, self.data)
                    continue

                with self._ref_lock:
                    if self._use_external_ref:
                        ref_deg = self._ref_deg
                    else:
                        ref_deg = self._reference_deg(self.sim_time)
                ref_rad = math.radians(ref_deg)

                q = self.data.qpos[0]
                qd = self.data.qvel[0]

                # PD -> 所需力矩 -> 拮抗肌肉激活 (含数值检查与屈曲限位保护)
                flex, ext = pd_to_activation(
                    ref_rad - q,
                    qd,
                    self.kp,
                    self.kd,
                    self.max_torque,
                    limit_reached=(q >= math.radians(self.limit_deg)),
                )

                self.data.ctrl[self.i_flex] = flex
                self.data.ctrl[self.i_ext] = ext
                mujoco.mj_step(self.model, self.data)
                self.sim_time += self.dt

            self._publish(self.data.ctrl[self.i_flex], self.data.ctrl[self.i_ext])

    def _publish(self, flex, ext):
        ang = Float64()
        ang.data = math.degrees(self.data.qpos[0])
        self._angle_pub.publish(ang)

        act = Float64MultiArray()
        act.data = [float(flex), float(ext)]
        self._act_pub.publish(act)

        trq = Float64()
        trq.data = float(self.data.qfrc_actuator[0])
        self._torque_pub.publish(trq)

    def _safe_stop(self):
        self.data.ctrl[self.i_flex] = 0.0
        self.data.ctrl[self.i_ext] = 0.0

    def shutdown(self):
        self._safe_stop()
        self.get_logger().info("已执行安全停止, 肌肉激活归零")


def main(args=None):
    rclpy.init(args=args)
    node = KneeTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
