#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多步态控制节点。

按时间调度生成站立/行走/快步/跑步的关节轨迹；切换时使用三次 Hermite
样条平滑过渡。发布目标关节角、浮动基座状态与步态信息。
"""

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

from biped_cubic_spline_gait_switch.config_loader import (
    build_gaits, build_model, build_schedule, default_params_path, load_config)
from biped_cubic_spline_gait_switch.gait import GaitPlanner


class ControllerNode(Node):
    def __init__(self):
        super().__init__("biped_controller")
        self.declare_parameter("config", default_params_path())
        self.declare_parameter("mode", "spline")
        self.declare_parameter("transition_time", 0.6)
        self.declare_parameter("control_rate", 200.0)

        cfg = load_config(self.get_parameter("config").value)
        model = build_model(cfg)
        mode = str(self.get_parameter("mode").value)
        transition = float(self.get_parameter("transition_time").value)
        self.planner = GaitPlanner(
            model, build_gaits(cfg), build_schedule(cfg),
            transition_time=transition, mode=mode,
            stand_foot_offset=float(cfg["control"]["stand_foot_offset"]),
        )
        self.mode = mode
        self.rate = float(self.get_parameter("control_rate").value)

        self.joint_pub = self.create_publisher(JointState, "/joint_cmd", 10)
        self.base_pub = self.create_publisher(Float32MultiArray, "/base_cmd", 10)
        self.info_pub = self.create_publisher(String, "/gait_info", 10)

        self.start_time = time.time()
        self.last_time = self.start_time
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            "步态控制节点启动：mode=%s，过渡 %.2f s，控制 %.0f Hz"
            % (self.mode, transition, self.rate))

    def tick(self):
        now = time.time()
        elapsed = now - self.start_time
        dt = min(max(now - self.last_time, 1e-4), 0.02)
        self.last_time = now

        state = self.planner.update(elapsed, dt)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["left_hip", "left_knee", "left_ankle",
                    "right_hip", "right_knee", "right_ankle"]
        msg.position = [float(v) for v in state["q"]]
        msg.velocity = [float(v) for v in state["dq"]]
        self.joint_pub.publish(msg)

        base_msg = Float32MultiArray()
        base_msg.data = [
            float(state["base_x"]), float(state["base_v"]),
            float(state["hip_z"]), float(state["lean"]),
        ]
        self.base_pub.publish(base_msg)

        info = {
            "time": round(elapsed, 4),
            "gait": state["gait"],
            "phase": round(state["phase"], 4),
            "switching": state["switching"],
            "switch_event": state["switch_event"],
            "progress": round(state["transition_progress"], 4),
            "base_x": round(state["base_x"], 4),
            "base_v": round(state["base_v"], 4),
            "contacts": state["contacts"],
            "feet": [[round(x, 4), round(z, 4)] for x, z in state["feet"]],
        }
        self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

        if state["switch_event"]:
            self.get_logger().info(
                "步态切换 -> %s（%.2f s）" % (state["gait"], elapsed))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
