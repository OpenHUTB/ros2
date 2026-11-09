# -*- coding: utf-8 -*-
"""ROS 2 节点：跑跌倒保护仿真，并把判定/冲击/关节状态发到话题上。

发布：
    /fall_protection/joint_states   sensor_msgs/JointState   15 个关节角
    /fall_protection/trigger        std_msgs/Bool            跌倒判据是否已触发
    /fall_protection/status         std_msgs/String          当前模式与倾角
    /fall_protection/impact         std_msgs/Float32MultiArray
                                    [头部, 躯干, 膝, 手] 的峰值接触力 (N)
订阅：
    /fall_protection/push           std_msgs/Float32         推扰力 (N)，正为向前推
    /fall_protection/strategy       std_msgs/String          rigid / limp / protect

仿真跑完一轮（4 s）后自动复位，方便反复观察；每次复位前把这一轮的峰值冲击
通过 /fall_protection/impact 发出去。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from . import config
from .protection import FallController
from .sim_core import FallSim


class FallProtectionNode(Node):
    def __init__(self):
        super().__init__("fall_protection_node")

        self.declare_parameter("sim_rate", config.DEFAULTS["sim_rate"])
        self.declare_parameter("publish_rate", config.DEFAULTS["publish_rate"])
        self.declare_parameter("tilt_threshold_deg", config.DEFAULTS["tilt_threshold_deg"])
        self.declare_parameter("tilt_rate_threshold", config.DEFAULTS["tilt_rate_threshold"])
        self.declare_parameter("detector_mode", config.DEFAULTS["detector_mode"])
        self.declare_parameter("protect_enable", config.DEFAULTS["protect_enable"])
        self.declare_parameter("protect_kp", config.DEFAULTS["protect_kp"])
        self.declare_parameter("protect_kv", config.DEFAULTS["protect_kv"])
        self.declare_parameter("protect_direction", config.DEFAULTS["protect_direction"])
        self.declare_parameter("push_force", config.DEFAULTS["push_force"])
        self.declare_parameter("push_duration", config.DEFAULTS["push_duration"])
        self.declare_parameter("push_delay", config.DEFAULTS["push_delay"])
        self.declare_parameter("strategy", "protect")
        self.declare_parameter("model_path", "")

        self.params = {k: self.get_parameter(k).value for k in (
            "sim_rate", "publish_rate", "tilt_threshold_deg", "tilt_rate_threshold",
            "detector_mode", "protect_enable", "protect_kp", "protect_kv",
            "protect_direction", "push_force", "push_duration", "push_delay")}
        self.strategy = str(self.get_parameter("strategy").value)
        model_path = str(self.get_parameter("model_path").value) or None

        self.sim = FallSim(model_path)
        self.sim.reset()
        self.controller = FallController(self.params)
        self.controller.reset(self.strategy)

        self.dt = self.sim.dt
        self.steps_per_tick = max(1, int(round(1.0 / (float(self.params["publish_rate"]) * self.dt))))
        self.t = 0.0
        self.push_force = float(self.params["push_force"])
        self.peak = {"head": 0.0, "torso": 0.0, "knee": 0.0, "hand": 0.0}

        self.pub_js = self.create_publisher(JointState, "fall_protection/joint_states", 10)
        self.pub_trigger = self.create_publisher(Bool, "fall_protection/trigger", 10)
        self.pub_status = self.create_publisher(String, "fall_protection/status", 10)
        self.pub_impact = self.create_publisher(Float32MultiArray, "fall_protection/impact", 10)

        self.create_subscription(Float32, "fall_protection/push", self._on_push, 10)
        self.create_subscription(String, "fall_protection/strategy", self._on_strategy, 10)

        # 控制周期 = 物理步长（500 Hz），发布按 publish_rate 分频
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            "fall_protection_node 启动：策略 %s，阈值 %.0f° / %.1f rad/s，dt %.3f s" % (
                self.strategy, self.params["tilt_threshold_deg"],
                self.params["tilt_rate_threshold"], self.dt))

    # ------------------------------------------------------------------ 回调
    def _on_push(self, msg):
        self.push_force = float(msg.data)
        self.get_logger().info("推扰力设为 %.0f N" % self.push_force)

    def _on_strategy(self, msg):
        name = str(msg.data).strip().lower()
        if name not in config.STRATEGIES:
            self.get_logger().warn("未知策略 %r，保持 %s" % (name, self.strategy))
            return
        self.strategy = name
        self.controller.reset(name)
        self.sim.reset()
        self.t = 0.0
        self.peak = {k: 0.0 for k in self.peak}
        self.get_logger().info("切换到策略 %s" % name)

    # ------------------------------------------------------------------ 主循环
    def _tick(self):
        push_start = float(self.params["push_delay"])
        push_dur = float(self.params["push_duration"])
        for _ in range(self.steps_per_tick):
            active = push_start <= self.t < push_start + push_dur
            self.sim.apply_push(self.push_force, active)
            state = self.controller.step(self.sim, self.t)
            self.sim.step()
            self.t += self.dt

            for part in self.peak:
                self.peak[part] = max(self.peak[part], self.sim.part_contact(part))

            if state["just_triggered"]:
                self.get_logger().warn(
                    "跌倒判据触发：t=%.3f s，倾角 %.1f°，角速度 %.2f rad/s，进入 %s" % (
                        self.t, state["tilt"] * 57.2958, state["tilt_rate"], state["mode"]))

        self._publish()
        if self.t > 4.0:
            self.get_logger().info(
                "本轮结束：头 %.0f N / 躯干 %.0f N / 膝 %.0f N" % (
                    self.peak["head"], self.peak["torso"], self.peak["knee"]))
            self.sim.reset()
            self.controller.reset(self.strategy)
            self.t = 0.0
            self.peak = {k: 0.0 for k in self.peak}

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(config.JOINT_NAMES)
        js.position = [self.sim.joint_pos(n) for n in config.JOINT_NAMES]
        js.velocity = [self.sim.joint_vel(n) for n in config.JOINT_NAMES]
        self.pub_js.publish(js)

        self.pub_trigger.publish(Bool(data=bool(self.controller.detector.triggered)))
        self.pub_status.publish(String(data="%s tilt=%.1fdeg head=%.3fm" % (
            self.controller._mode_name(self.controller.detector.triggered),
            self.sim.tilt() * 57.2958, self.sim.head_height())))
        self.pub_impact.publish(Float32MultiArray(data=[
            float(self.peak["head"]), float(self.peak["torso"]),
            float(self.peak["knee"]), float(self.peak["hand"])]))


def main(args=None):
    rclpy.init(args=args)
    node = FallProtectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
