#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：虚拟通道测距仿真 + 双侧墙居中 PD 巡航控制器。

为避免引入额外消息包，话题均使用 ``std_msgs/String`` 承载 JSON：

| 话题 | 方向 | 内容 |
| --- | --- | --- |
| ``/corridor/scan`` | 仿真 -> 控制器 | ``{"left": 0.9, "right": 1.1, "forward": 6.0}`` |
| ``/corridor/cmd_vel`` | 控制器 -> 仿真器 | ``{"linear": 0.2, "angular": 0.05}`` |
| ``/corridor/state`` | 控制器 -> 观测 | ``{"state": "CENTER", "linear": ..., "angular": ...}`` |

``centering.py`` 核心不依赖 rclpy，可直接用系统 Python 做单元测试；
本文件只负责 ROS 2 收发与参数装载。
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .centering import CenteringParams, CorridorCenteringController, diff_drive_step

CORRIDOR_HALF_WIDTH = 1.0   # 通道半宽（m），左右墙分别在 y = ±1.0
CORRIDOR_LENGTH = 14.0      # 通道总长（m）


class ScanSimNode(Node):
    """2D 运动学积分的虚拟通道，按当前位姿推算左右墙与前方距离并发布。"""

    def __init__(self) -> None:
        super().__init__("corridor_scan_sim")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("start_x", 0.3)
        self.declare_parameter("start_y", 0.55)
        self.declare_parameter("start_yaw", -0.18)

        rate = self.get_parameter("rate").value
        self._pose = (
            float(self.get_parameter("start_x").value),
            float(self.get_parameter("start_y").value),
            float(self.get_parameter("start_yaw").value),
        )
        self._cmd = (0.0, 0.0)  # (v, w)，由控制器回给仿真器
        self._last_t = self.get_clock().now()

        self._pub = self.create_publisher(String, "/corridor/scan", 10)
        self._sub = self.create_subscription(
            String, "/corridor/cmd_vel", self._on_cmd, 10)
        self._timer = self.create_timer(1.0 / float(rate), self._tick)

    def _on_cmd(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._cmd = (float(data.get("linear", 0.0)),
                         float(data.get("angular", 0.0)))
        except (json.JSONDecodeError, TypeError, ValueError):
            self.get_logger().warn("收到无法解析的 cmd_vel，按零速处理")
            self._cmd = (0.0, 0.0)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds / 1e9
        self._last_t = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / 20.0

        v, w = self._cmd
        self._pose = diff_drive_step(self._pose, v, w, dt)
        x, y, yaw = self._pose

        d_left = CORRIDOR_HALF_WIDTH - y
        d_right = CORRIDOR_HALF_WIDTH + y
        # 前方沿航向射线到通道端墙的距离（小角度近似）。
        forward = max(0.0, (CORRIDOR_LENGTH - x) / max(math.cos(yaw), 0.2))

        msg = String()
        msg.data = json.dumps({
            "left": round(d_left, 4),
            "right": round(d_right, 4),
            "forward": round(forward, 4),
        })
        self._pub.publish(msg)


class CenteringNode(Node):
    """订阅虚拟测距，运行居中 PD 控制器，发布速度指令与状态。"""

    def __init__(self) -> None:
        super().__init__("corridor_centering")
        self.declare_parameter("kp", 1.2)
        self.declare_parameter("kd", 0.6)
        self.declare_parameter("v_max", 0.4)
        self.declare_parameter("v_floor", 0.08)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("forward_safety", 0.6)
        self.declare_parameter("forward_recover", 0.9)
        self.declare_parameter("scan_timeout", 0.5)

        params = CenteringParams(
            kp=float(self.get_parameter("kp").value),
            kd=float(self.get_parameter("kd").value),
            v_max=float(self.get_parameter("v_max").value),
            v_floor=float(self.get_parameter("v_floor").value),
            w_max=float(self.get_parameter("w_max").value),
            forward_safety=float(self.get_parameter("forward_safety").value),
            forward_recover=float(self.get_parameter("forward_recover").value),
            scan_timeout=float(self.get_parameter("scan_timeout").value),
        )
        self._controller = CorridorCenteringController(params)

        self._cmd_pub = self.create_publisher(String, "/corridor/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/corridor/state", 10)
        self._sub = self.create_subscription(
            String, "/corridor/scan", self._on_scan, 10)
        self._last_scan_time: float | None = None
        self._timeout_timer = self.create_timer(1.0 / 10.0, self._on_timeout)

    def _on_scan(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            d_left = float(data["left"])
            d_right = float(data["right"])
            d_forward = float(data["forward"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warn("收到无法解析的 scan，跳过本轮")
            return

        now = self.get_clock().now().nanoseconds / 1e9
        self._last_scan_time = now
        result = self._controller.step(d_left, d_right, d_forward, now)

        cmd = String()
        cmd.data = json.dumps({
            "linear": round(result["v"], 4),
            "angular": round(result["w"], 4),
        })
        self._cmd_pub.publish(cmd)

        state = String()
        state.data = json.dumps({
            "state": result["state"],
            "linear": round(result["v"], 4),
            "angular": round(result["w"], 4),
            "left": round(d_left, 4),
            "right": round(d_right, 4),
            "forward": round(d_forward, 4),
            "error": round(result["e"], 4),
        })
        self._state_pub.publish(state)

    def _on_timeout(self) -> None:
        """10 Hz 看门狗：距上次测距超过 scan_timeout 时零速停车并置 BLOCKED。"""
        now = self.get_clock().now().nanoseconds / 1e9
        if (self._last_scan_time is not None
                and now - self._last_scan_time > self._controller.params.scan_timeout):
            cmd = String()
            cmd.data = json.dumps({"linear": 0.0, "angular": 0.0})
            self._cmd_pub.publish(cmd)
            self._controller.state = "BLOCKED"


def scan_main() -> None:
    rclpy.init()
    node = ScanSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def centering_main() -> None:
    rclpy.init()
    node = CenteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
